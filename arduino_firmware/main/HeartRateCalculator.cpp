#include "HeartRateCalculator.h"
#include <algorithm>
#include <numeric>
#include <deque>

HeartRateCalculator::HeartRateCalculator(uint16_t threshold)
    : _threshold(threshold),
      _bpm(0),
      _hasValidBPM(false),
      _peakCount(0) {
    _samples.reserve(2000); // ~20s at 100Hz
}

void HeartRateCalculator::addSample(unsigned long timestamp, uint16_t value) {
    _samples.push_back({timestamp, value});
}

void HeartRateCalculator::clearSamples() {
    _samples.clear();
    _recentIntervals.clear();
    _hasValidBPM = false;
    _peakCount = 0;
}

size_t HeartRateCalculator::getSampleCount() const { return _samples.size(); }
float HeartRateCalculator::getBPM() const { return _bpm; }
bool HeartRateCalculator::hasValidBPM() const { return _hasValidBPM; }
void HeartRateCalculator::setThreshold(uint16_t threshold) { _threshold = threshold; }
uint16_t HeartRateCalculator::getPeakCount() const { return _peakCount; }

uint16_t HeartRateCalculator::getSampleValue(size_t index) const {
    if (index >= _samples.size()) return 0;
    return _samples[index].value;
}

void HeartRateCalculator::printSampleStats() const {
    if (_samples.empty()) {
        Serial.println("  No samples collected");
        return;
    }

    Serial.println("  Sample values (first 20):");
    for (size_t i = 0; i < 20 && i < _samples.size(); i++) {
        if (i % 5 == 0) Serial.print("    ");
        Serial.print(_samples[i].value);
        Serial.print(" ");
        if ((i + 1) % 5 == 0) Serial.println();
    }
    if (_samples.size() % 5 != 0) Serial.println();
}


std::vector<unsigned long> HeartRateCalculator::findPeaksAdaptive(uint16_t threshold) {
    std::vector<unsigned long> peaks;
    if (_samples.size() < 5) return peaks;

    // Compute dynamic refractory (default 350 ms)
    unsigned long dynamicRefractory = 350;
    if (_recentIntervals.size() >= 5) {
        std::vector<unsigned long> tmp = _recentIntervals;
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        unsigned long medianInt = tmp[tmp.size() / 2];
        dynamicRefractory = std::min(600UL, std::max(250UL, (unsigned long)(0.45 * medianInt)));
    }

    for (size_t i = 2; i < _samples.size() - 2; i++) {
        uint16_t val = _samples[i].value;
        if (val > threshold &&
            val > _samples[i - 1].value &&
            val > _samples[i - 2].value &&
            val > _samples[i + 1].value &&
            val > _samples[i + 2].value) {

            if (peaks.empty() || (_samples[i].timestamp - peaks.back() > dynamicRefractory)) {
                peaks.push_back(_samples[i].timestamp);
            }
        }
    }

    Serial.print("  Dynamic refractory: ");
    Serial.print(dynamicRefractory);
    Serial.println(" ms");
    return peaks;
}

float HeartRateCalculator::calculateBPM() {
    // Need enough data (≥ 10s @ 100 Hz → 1000 samples is nice; but allow lower)
    if (_samples.size() < 300) { // ~3 s minimum
        _hasValidBPM = false; _bpm = 0; return 0;
    }

    // ---- 0) Basic stats for logs ----
    uint16_t minVal = 65535, maxVal = 0;
    double sum = 0.0;
    for (auto &s : _samples) {
        if (s.value < minVal) minVal = s.value;
        if (s.value > maxVal) maxVal = s.value;
        sum += s.value;
    }
    float range = maxVal - minVal;
    float avg = sum / _samples.size();

    Serial.print(" Signal: min="); Serial.print(minVal);
    Serial.print(" max=");         Serial.print(maxVal);
    Serial.print(" avg=");         Serial.print(avg, 2);
    Serial.print(" range=");       Serial.println(range, 0);

    if (range < 60) { // too flat
        Serial.println(" Weak signal — skipping BPM calculation");
        _hasValidBPM = false; _bpm = 0; return 0;
    }

    // ---- 1) Band-pass filter (0.7–5 Hz) for fs=100 Hz ----
    const float fs = 100.0f, dt = 1.0f / fs;
    const float fc_hp = 0.7f, fc_lp = 5.0f;
    const float RC_hp = 1.0f / (2.0f * 3.1415926f * fc_hp);
    const float RC_lp = 1.0f / (2.0f * 3.1415926f * fc_lp);
    const float a_hp = RC_hp / (RC_hp + dt);    // y[n] = a*(y[n-1] + x[n]-x[n-1])
    const float a_lp = dt / (RC_lp + dt);       // y[n] += a*(x - y)

    std::vector<float> bp(_samples.size());
    float yhp = 0.0f, ylp = 0.0f, xprev = (float)_samples[0].value;
    for (size_t i = 0; i < _samples.size(); ++i) {
        float x = (float)_samples[i].value;
        yhp = a_hp * (yhp + x - xprev);
        xprev = x;
        ylp += a_lp * (yhp - ylp);
        bp[i] = ylp;
    }

    // ---- 2) Robust z-normalization (median/MAD) ----
    std::vector<float> tmp = bp;
    std::sort(tmp.begin(), tmp.end());
    float med = tmp[tmp.size()/2];

    std::vector<float> dev(tmp.size());
    for (size_t i = 0; i < tmp.size(); ++i) dev[i] = fabsf(bp[i] - med);
    std::sort(dev.begin(), dev.end());
    float mad = dev[dev.size()/2];
    float scale = (mad < 1e-3f) ? 1.0f : (1.4826f * mad);

    std::vector<float> z(bp.size());
    for (size_t i = 0; i < bp.size(); ++i) z[i] = (bp[i] - med) / scale;

    // ---- 3) Candidate peak detection (stricter) ----
    std::vector<float> d(bp.size(), 0.0f);
    for (size_t i = 1; i + 1 < bp.size(); ++i) d[i] = 0.5f * (bp[i+1] - bp[i-1]);

    float zThresh = 0.8f;
    if (range < 150) zThresh = 0.9f;
    if (range > 350) zThresh = 0.6f;

    const unsigned long MIN_REFR_MS = 420;
    const unsigned long MAX_REFR_MS = 1200;
    unsigned long dynRefr = 450;

    struct Peak { size_t idx; unsigned long t; float z; };
    std::vector<Peak> cand;

    for (size_t i = 2; i + 2 < z.size(); ++i) {
        bool localMax = (bp[i] > bp[i-1] && bp[i] > bp[i+1]);
        bool slopeFlip = (d[i-1] > 0.0f && d[i+1] < 0.0f);
        if (!localMax || !slopeFlip) continue;
        if (z[i] < zThresh) continue;

        size_t w = 15;
        size_t a = (i > w ? i - w : 0), b = std::min(z.size()-1, i + w);
        std::vector<float> win(z.begin()+a, z.begin()+b+1);
        std::nth_element(win.begin(), win.begin()+win.size()/2, win.end());
        float locMed = win[win.size()/2];
        if (z[i] < locMed + 0.6f) continue;

        cand.push_back({ i, _samples[i].timestamp, z[i] });
    }

    if (cand.size() < 2) { _hasValidBPM = false; _bpm = 0; return 0; }

    // ---- 4) Merge-close-peaks (< 450 ms) → keep stronger ----
    std::vector<Peak> peaks;
    for (size_t k = 0; k < cand.size(); ++k) {
        if (peaks.empty()) { peaks.push_back(cand[k]); continue; }
        unsigned long dt_ms = cand[k].t - peaks.back().t;
        if (dt_ms < 450) {
            if (cand[k].z > peaks.back().z) peaks.back() = cand[k];
        } else {
            peaks.push_back(cand[k]);
        }
    }

    // Extra guard: drop residual close pairs
    std::vector<Peak> peaks2;
    for (size_t i = 0; i < peaks.size(); ++i) {
        if (peaks2.empty()) { peaks2.push_back(peaks[i]); continue; }
        unsigned long dt_ms = peaks[i].t - peaks2.back().t;
        if (dt_ms < MIN_REFR_MS) {
            if (peaks[i].z > peaks2.back().z) peaks2.back() = peaks[i];
        } else {
            peaks2.push_back(peaks[i]);
        }
    }
    peaks.swap(peaks2);

    Serial.print(" Peaks found (after merge): ");
    Serial.println(peaks.size());
    if (peaks.size() < 3) { _hasValidBPM = false; _bpm = 0; return 0; }

    // ---- 5) RR intervals & BPM ----
    std::vector<float> rr;
    rr.reserve(peaks.size() - 1);
    for (size_t i = 1; i < peaks.size(); ++i)
        rr.push_back((float)(peaks[i].t - peaks[i-1].t));

    // Store RR intervals for HRV before sorting
    _lastRR = rr;
    Serial.print("[DBG] Stored RR intervals: ");
    Serial.println(_lastRR.size());

    std::sort(rr.begin(), rr.end());
    float rr_med = rr[rr.size()/2];
    float bpm_median = 60000.0f / rr_med;

    Serial.print(" Median interval: "); Serial.print(rr_med, 2);
    Serial.print(" ms -> BPM_med = "); Serial.println(bpm_median, 2);

    // ---- 6) Autocorrelation fallback (recover fundamental if doubled) ----
    size_t N = z.size();
    size_t tail = (N > 1200 ? 1200 : N);
    size_t s0 = N - tail;

    auto acf_at = [&](int lag) -> double {
        double mean = 0.0;
        for (size_t i = s0; i < s0 + tail; ++i) mean += z[i];
        mean /= (double)tail;

        double num = 0.0, den0 = 0.0, den1 = 0.0;
        for (size_t i = s0; i + lag < s0 + tail; ++i) {
            double a = z[i] - mean;
            double b = z[i+lag] - mean;
            num  += a * b;
            den0 += a * a;
            den1 += b * b;
        }
        double den = sqrt(den0 * den1) + 1e-9;
        return num / den;
    };

    int bestLag = -1; double bestR = -1e9;
    for (int lag = 45; lag <= 150; ++lag) {
        double r = acf_at(lag);
        if (r > bestR) { bestR = r; bestLag = lag; }
    }
    float bpm_acf = (bestLag > 0) ? (6000.0f / bestLag) : 0.0f;

    Serial.print(" ACF lag="); Serial.print(bestLag);
    Serial.print(" r=");      Serial.print(bestR, 3);
    Serial.print(" -> BPM_acf = "); Serial.println(bpm_acf, 2);

    // ---- 7) Decision ----
    float bpm_final = bpm_median;
    if (bpm_median > 95.0f && bpm_acf >= 50.0f && bpm_acf <= 95.0f) {
        if (fabsf(bpm_median - 2.0f * bpm_acf) / bpm_median < 0.20f) {
            Serial.println(" Using ACF fallback (doubled peak suspected).");
            bpm_final = bpm_acf;
        }
    }

    if (bpm_final < 40 || bpm_final > 180) {
        Serial.println("[HEART] BPM: INVALID");
        _hasValidBPM = false; _bpm = 0; return 0;
    }

    static bool hadValid = false;
    if (hadValid) _bpm = 0.7f * _bpm + 0.3f * bpm_final;
    else _bpm = bpm_final;

    _hasValidBPM = true;
    hadValid = true;
    Serial.print(" BPM_final = "); Serial.println(_bpm, 1);
    return _bpm;
}


HeartRateCalculator::HRVResult HeartRateCalculator::calculateHRV() {
    HRVResult result = {0, 0, 0};

    // Need at least a few intervals to even try
    if (_lastRR.size() < 3) {
        Serial.println("[HRV] Not enough RR intervals");
        return result;
    }

    // --- 1) Start from RR copy ---
    std::vector<float> rr = _lastRR;

    // --- 2) Global plausibility gate (ms) ---
    const float RR_MIN = 400.0f;
    const float RR_MAX = 1200.0f;
    std::vector<float> rr_rng;
    rr_rng.reserve(rr.size());
    for (float r : rr) if (r >= RR_MIN && r <= RR_MAX) rr_rng.push_back(r);

    // --- 3) Local median consistency gate (remove spikes/outliers) ---
    // Sliding window median (w=5). Keep r if within ±max(20%, 120 ms) of local median.
    std::vector<float> rr_nn;
    rr_nn.reserve(rr_rng.size());
    const int W = 5;
    for (size_t i = 0; i < rr_rng.size(); ++i) {
        size_t a = (i >= (size_t)W ? i - W : 0);
        size_t b = std::min(rr_rng.size() - 1, i + (size_t)W);
        std::vector<float> win(rr_rng.begin() + a, rr_rng.begin() + b + 1);
        std::nth_element(win.begin(), win.begin() + win.size() / 2, win.end());
        float med = win[win.size() / 2];
        float tol = std::max(0.20f * med, 120.0f); // ±20% or ±120 ms
        if (fabsf(rr_rng[i] - med) <= tol) rr_nn.push_back(rr_rng[i]);
    }

    // --- 4) If very few remain, bail out ---
    int removed = (int)rr.size() - (int)rr_nn.size();
    if (rr_nn.size() < 5) {
        Serial.print("[HRV] Too few clean NN intervals (kept ");
        Serial.print(rr_nn.size());
        Serial.print(" / ");
        Serial.print(rr.size());
        Serial.println("). HRV unreliable.");
        return result;
    }

    Serial.print("[HRV] Cleaned NN count: ");
    Serial.print(rr_nn.size());
    Serial.print(" (removed ");
    Serial.print(removed);
    Serial.println(" outliers)");

    // --- 5) SDNN ---
    float mean = std::accumulate(rr_nn.begin(), rr_nn.end(), 0.0f) / rr_nn.size();
    float sumSq = 0.0f;
    for (float x : rr_nn) { float d = x - mean; sumSq += d * d; }
    result.SDNN = sqrtf(sumSq / (rr_nn.size() - 1));

    // --- 6) RMSSD + pNN50 ---
    std::vector<float> diffs;
    diffs.reserve(rr_nn.size() - 1);
    for (size_t i = 1; i < rr_nn.size(); ++i) diffs.push_back(rr_nn[i] - rr_nn[i - 1]);

    float sumDiffSq = 0.0f;
    int nn50 = 0;
    for (float d : diffs) {
        sumDiffSq += d * d;
        if (fabsf(d) > 50.0f) nn50++;
    }
    result.RMSSD = sqrtf(sumDiffSq / diffs.size());
    result.pNN50 = (100.0f * nn50) / diffs.size();

    Serial.println("[HRV] Metrics (cleaned NN):");
    Serial.print("  SDNN: ");  Serial.print(result.SDNN, 1);  Serial.println(" ms");
    Serial.print("  RMSSD: "); Serial.print(result.RMSSD, 1); Serial.println(" ms");
    Serial.print("  pNN50: "); Serial.print(result.pNN50, 1); Serial.println(" %");

    return result;
}
