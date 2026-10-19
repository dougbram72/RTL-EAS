// Unattended-service outputs for the command-line decoder: an append-only
// JSON-lines log of decoded headers and finished alert clips, a periodically
// rewritten receiver status file, and one WAV clip per alert (pre-roll through
// the end-of-message burst). Consumers such as a map server tail the log and
// read the status; neither is touched unless its path is set.
#pragma once

#include "app/options.h"
#include "eas/same_message.h"
#include "util/wav_io.h"

#include <chrono>
#include <deque>
#include <mutex>
#include <string>

class AlertService
{
public:
    explicit AlertService(const Options& opt);
    ~AlertService();

    bool enabled() const { return !logPath_.empty() || !statusPath_.empty() || !clipsDir_.empty(); }

    // SDR / decode thread.
    void onAudio(const float* a, int n, double sampleRate);
    void onMessage(const SameMessage& m);
    void onEom();

    // Main thread, called often; writes the status file every 60 s.
    void tick(double channelPowerDb);

    // Closes any open clip and writes a final status.
    void finish(double channelPowerDb);

private:
    void finishClipLocked(const char* endedBy);
    void appendLogLocked(const std::string& line);
    void writeStatusLocked(double channelPowerDb);

    std::string logPath_, statusPath_, clipsDir_, device_;
    double      freqMHz_    = 0.0;
    double      clipMaxSec_ = 300.0;

    std::mutex mtx_;
    double     rate_     = 0.0;
    double     audioSec_ = 0.0;

    std::deque<float> preroll_;
    size_t            prerollCap_ = 0;

    WavWriter   clip_;
    std::string clipId_, clipFile_;
    double      clipStart_  = 0.0;
    double      clipStopAt_ = -1.0;

    double   sumSq_   = 0.0;
    uint64_t sumN_    = 0;
    uint64_t headers_ = 0;
    uint64_t eoms_    = 0;

    std::chrono::steady_clock::time_point nextStatus_{};
};
