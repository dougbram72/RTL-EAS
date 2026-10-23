#include "app/alert_service.h"

#include "util/log.h"
#include "version.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <vector>

namespace
{

constexpr double kPrerollSec = 20.0; // covers the three headers (~10 s) + the decoder's 6 s grouping delay
constexpr double kEomTailSec = 2.0;
constexpr int    kStatusSec  = 60;

std::tm utcNow(std::time_t t)
{
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    return tm;
}

std::string isoUtc(std::time_t t)
{
    std::tm tm = utcNow(t);
    char    buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

std::string compactUtc(std::time_t t)
{
    std::tm tm = utcNow(t);
    char    buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%dT%H%M%SZ", &tm);
    return buf;
}

// Event codes are letters only once parsed, but keep ids filename-safe anyway.
std::string safeToken(const std::string& s)
{
    std::string out;
    for (char c : s)
    {
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
        {
            out.push_back(c);
        }
    }
    return out.empty() ? "UNK" : out;
}

std::string jsonNumber(double v, const char* fmt)
{
    if (!std::isfinite(v))
    {
        return "null";
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return buf;
}

} // namespace

AlertService::AlertService(const Options& opt)
    : logPath_(opt.logPath), statusPath_(opt.statusPath), clipsDir_(opt.clipsDir), freqMHz_(opt.freqMHz),
      clipMaxSec_(opt.clipMaxSec > 0.0 ? opt.clipMaxSec : 300.0)
{
    device_ = opt.deviceSerial.empty() ? std::to_string(opt.device) : "serial:" + opt.deviceSerial;
    if (!clipsDir_.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(clipsDir_, ec);
    }
}

AlertService::~AlertService()
{
    std::lock_guard<std::mutex> lk(mtx_);
    finishClipLocked("shutdown");
}

void AlertService::onAudio(const float* a, int n, double sampleRate)
{
    if (n <= 0)
    {
        return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (rate_ <= 0.0 && sampleRate > 0.0)
    {
        rate_       = sampleRate;
        prerollCap_ = (size_t)(kPrerollSec * rate_);
    }
    if (rate_ <= 0.0)
    {
        return;
    }
    audioSec_ += (double)n / rate_;

    for (int i = 0; i < n; ++i)
    {
        sumSq_ += (double)a[i] * a[i];
    }
    sumN_ += (uint64_t)n;

    if (!clipsDir_.empty())
    {
        preroll_.insert(preroll_.end(), a, a + n);
        while (preroll_.size() > prerollCap_)
        {
            preroll_.pop_front();
        }
    }

    if (clip_.isOpen())
    {
        clip_.write(a, n);
        if (clipStopAt_ >= 0.0 && audioSec_ >= clipStopAt_)
        {
            finishClipLocked("eom");
        }
        else if (audioSec_ - clipStart_ >= clipMaxSec_)
        {
            finishClipLocked("max");
        }
    }
}

void AlertService::onMessage(const SameMessage& m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++headers_;
    std::time_t now = std::time(nullptr);
    std::string id  = compactUtc(now) + "-" + safeToken(m.event);

    finishClipLocked("next");

    std::string clipName;
    if (!clipsDir_.empty() && rate_ > 0.0)
    {
        std::string path = (std::filesystem::path(clipsDir_) / (id + ".wav")).string();
        std::string err;
        if (clip_.open(path, (int)std::lround(rate_), 1, err))
        {
            std::vector<float> pr(preroll_.begin(), preroll_.end());
            if (!pr.empty())
            {
                clip_.write(pr.data(), (int)pr.size());
            }
            clipId_     = id;
            clipFile_   = id + ".wav";
            clipStart_  = audioSec_ - (double)pr.size() / rate_;
            clipStopAt_ = -1.0;
            clipName    = clipFile_;
        }
        else
        {
            logWrite(LogLevel::Info, "clip open failed: %s", err.c_str());
        }
    }

    std::string line = sameJsonMessage(m);
    line.pop_back(); // trailing '}'
    line += ",\"id\":\"" + id + "\",\"received\":\"" + isoUtc(now) + "\"";
    if (!clipName.empty())
    {
        line += ",\"clip\":\"" + clipName + "\"";
    }
    line += "}";
    appendLogLocked(line);
}

void AlertService::onEom()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++eoms_;
    if (clip_.isOpen() && clipStopAt_ < 0.0)
    {
        clipStopAt_ = audioSec_ + kEomTailSec;
    }
}

void AlertService::finishClipLocked(const char* endedBy)
{
    if (!clip_.isOpen())
    {
        return;
    }
    clip_.close();
    double secs = audioSec_ - clipStart_;
    appendLogLocked("{\"type\":\"clip\",\"id\":\"" + clipId_ + "\",\"file\":\"" + clipFile_ +
                    "\",\"seconds\":" + jsonNumber(secs, "%.1f") + ",\"endedBy\":\"" + endedBy + "\"}");
    logWrite(LogLevel::Info, "alert clip %s saved (%.1f s, %s)", clipFile_.c_str(), secs, endedBy);
    clipId_.clear();
    clipFile_.clear();
    clipStopAt_ = -1.0;
}

void AlertService::appendLogLocked(const std::string& line)
{
    if (logPath_.empty())
    {
        return;
    }
    std::FILE* f = std::fopen(logPath_.c_str(), "ab");
    if (!f)
    {
        logWrite(LogLevel::Info, "cannot append to %s", logPath_.c_str());
        return;
    }
    std::fprintf(f, "%s\n", line.c_str());
    std::fclose(f);
}

void AlertService::writeStatusLocked(double channelPowerDb)
{
    if (statusPath_.empty())
    {
        return;
    }
    double rms = sumN_ ? std::sqrt(sumSq_ / (double)sumN_) : NAN;
    sumSq_     = 0.0;
    sumN_      = 0;

    std::string json = std::string("{\"type\":\"status\",\"time\":\"") + isoUtc(std::time(nullptr)) +
                       "\",\"version\":\"" + RTLEAS_VERSION + "\",\"device\":\"" + device_ +
                       "\",\"freqMHz\":" + jsonNumber(freqMHz_, "%.4f") +
                       ",\"channelPowerDb\":" + jsonNumber(channelPowerDb, "%.1f") +
                       ",\"audioRms\":" + jsonNumber(rms, "%.4f") + ",\"headers\":" + std::to_string(headers_) +
                       ",\"eoms\":" + std::to_string(eoms_) + ",\"recording\":" + (clip_.isOpen() ? "true" : "false") +
                       "}\n";

    std::string tmp = statusPath_ + ".tmp";
    std::FILE*  f   = std::fopen(tmp.c_str(), "wb");
    if (!f)
    {
        return;
    }
    std::fputs(json.c_str(), f);
    std::fclose(f);
    std::error_code ec;
    std::filesystem::rename(tmp, statusPath_, ec);
}

void AlertService::tick(double channelPowerDb)
{
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mtx_);
    if (now < nextStatus_)
    {
        return;
    }
    nextStatus_ = now + std::chrono::seconds(kStatusSec);
    writeStatusLocked(channelPowerDb);
}

void AlertService::finish(double channelPowerDb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    finishClipLocked("shutdown");
    writeStatusLocked(channelPowerDb);
}
