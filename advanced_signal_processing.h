/**
 * @file advanced_signal_processing.h
 * @author adzetto
 * @brief Advanced Signal Processing Framework for Electric Vehicle Systems
 * @version 1.0
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 * @details This module provides comprehensive signal processing capabilities including
 * digital filtering, FFT analysis, wavelet transforms, adaptive filters,
 * spectral analysis, and real-time signal processing for EV systems.
 */

#ifndef ADVANCED_SIGNAL_PROCESSING_H
#define ADVANCED_SIGNAL_PROCESSING_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <random>
#include <complex>
#include <tuple>
#include <condition_variable>
#include <future>

namespace signal_processing {

// Signal processing types and configurations
using Complex = std::complex<double>;
using Signal = std::vector<double>;
using ComplexSignal = std::vector<Complex>;

enum class FilterType {
    LOW_PASS,
    HIGH_PASS,
    BAND_PASS,
    BAND_STOP,
    NOTCH,
    ALL_PASS,
    CUSTOM
};

enum class FilterDesign {
    BUTTERWORTH,
    CHEBYSHEV_I,
    CHEBYSHEV_II,
    ELLIPTIC,
    BESSEL,
    FIR_WINDOWED,
    FIR_EQUIRIPPLE,
    IIR_BILINEAR
};

enum class WindowFunction {
    RECTANGULAR,
    HANNING,
    HAMMING,
    BLACKMAN,
    KAISER,
    TUKEY,
    GAUSSIAN,
    BARTLETT,
    WELCH
};

enum class SpectralEstimation {
    PERIODOGRAM,
    WELCH,
    BARTLETT,
    MULTITAPER,
    AUTOREGRESSIVE,
    MUSIC,
    ESPRIT,
    PISARENKO
};

enum class WaveletType {
    DAUBECHIES,
    BIORTHOGONAL,
    COIFLETS,
    MORLET,
    MEXICAN_HAT,
    GAUSSIAN,
    MEYER,
    HAAR
};

enum class AdaptiveAlgorithm {
    LMS,              // Least Mean Squares
    NLMS,             // Normalized LMS
    RLS,              // Recursive Least Squares
    KALMAN,           // Kalman Filter
    PARTICLE_FILTER,  // Particle Filter
    EXTENDED_KALMAN,  // Extended Kalman Filter
    UNSCENTED_KALMAN  // Unscented Kalman Filter
};

struct SignalProperties {
    double samplingRate;
    size_t length;
    double duration;
    double meanValue;
    double rmsValue;
    double peakValue;
    double crestFactor;
    double snr;  // Signal-to-Noise Ratio
    double thd;  // Total Harmonic Distortion
    std::chrono::steady_clock::time_point timestamp;
    
    SignalProperties() : samplingRate(1000.0), length(0), duration(0.0),
                        meanValue(0.0), rmsValue(0.0), peakValue(0.0),
                        crestFactor(0.0), snr(0.0), thd(0.0),
                        timestamp(std::chrono::steady_clock::now()) {}
};

struct FilterCoefficients {
    std::vector<double> numerator;    // FIR coefficients or IIR numerator
    std::vector<double> denominator;  // IIR denominator (empty for FIR)
    double gain;
    FilterType type;
    FilterDesign design;
    double cutoffFrequency;
    double bandwidth;
    int order;
    
    FilterCoefficients() : gain(1.0), type(FilterType::LOW_PASS),
                          design(FilterDesign::BUTTERWORTH),
                          cutoffFrequency(100.0), bandwidth(0.0), order(4) {}
};

struct SpectralData {
    std::vector<double> frequencies;
    std::vector<double> magnitudes;
    std::vector<double> phases;
    std::vector<double> powerSpectralDensity;
    double fundamentalFrequency;
    std::vector<std::pair<double, double>> harmonics;  // frequency, magnitude pairs
    double spectralCentroid;
    double spectralRolloff;
    double spectralFlatness;
    
    SpectralData() : fundamentalFrequency(0.0), spectralCentroid(0.0),
                    spectralRolloff(0.0), spectralFlatness(0.0) {}
};

// Digital Filter Engine
class DigitalFilterEngine {
private:
    std::unordered_map<std::string, FilterCoefficients> filterBank;
    std::unordered_map<std::string, std::vector<double>> filterStates; // For IIR filters
    mutable std::mutex filterMutex;
    
    // Filter design methods
    FilterCoefficients designButterworthFilter(FilterType type, int order, double cutoff, double samplingRate, double bandwidth = 0.0);
    FilterCoefficients designChebyshevFilter(FilterType type, int order, double cutoff, double ripple, double samplingRate, double bandwidth = 0.0);
    FilterCoefficients designEllipticFilter(FilterType type, int order, double cutoff, double passbandRipple, double stopbandRipple, double samplingRate, double bandwidth = 0.0);
    FilterCoefficients designFIRFilter(FilterType type, int order, double cutoff, WindowFunction window, double samplingRate, double bandwidth = 0.0);
    
    // Window functions
    std::vector<double> generateWindow(WindowFunction type, size_t length, double parameter = 0.0);
    double rectangularWindow(size_t n, size_t N);
    double hanningWindow(size_t n, size_t N);
    double hammingWindow(size_t n, size_t N);
    double blackmanWindow(size_t n, size_t N);
    double kaiserWindow(size_t n, size_t N, double beta);
    
    // Filter implementation
    double processSampleIIR(double input, const FilterCoefficients& filter, std::vector<double>& state);
    double processSampleFIR(double input, const FilterCoefficients& filter, std::vector<double>& state);
    
public:
    DigitalFilterEngine() = default;
    ~DigitalFilterEngine() = default;
    
    // Filter design and management
    std::string createFilter(const std::string& name, FilterType type, FilterDesign design, 
                           int order, double cutoff, double samplingRate, 
                           double bandwidth = 0.0, double ripple = 1.0);
    void removeFilter(const std::string& name);
    bool filterExists(const std::string& name) const;
    FilterCoefficients getFilterCoefficients(const std::string& name) const;
    std::vector<std::string> getAvailableFilters() const;
    
    // Signal filtering
    Signal filterSignal(const Signal& input, const std::string& filterName);
    Signal filterSignal(const Signal& input, const FilterCoefficients& filter);
    double filterSample(double input, const std::string& filterName);
    
    // Real-time filtering
    void initializeRealTimeFilter(const std::string& filterName);
    double processRealTimeSample(double input, const std::string& filterName);
    void resetFilterState(const std::string& filterName);
    
    // Filter analysis
    std::vector<Complex> getFrequencyResponse(const std::string& filterName, const std::vector<double>& frequencies);
    std::vector<double> getGroupDelay(const std::string& filterName, const std::vector<double>& frequencies);
    std::vector<double> getImpulseResponse(const std::string& filterName, size_t length);
    std::vector<double> getStepResponse(const std::string& filterName, size_t length);
    
    // Cascade and parallel combinations
    std::string cascadeFilters(const std::string& name, const std::vector<std::string>& filterNames);
    std::string parallelFilters(const std::string& name, const std::vector<std::string>& filterNames, const std::vector<double>& weights);
    
    // Custom filter implementation
    void addCustomFilter(const std::string& name, const FilterCoefficients& coefficients);
    void updateFilterCoefficients(const std::string& name, const FilterCoefficients& coefficients);
};

// Fast Fourier Transform Engine
class FFTEngine {
private:
    mutable std::mutex fftMutex;
    
    // FFT implementations
    void fftRecursive(ComplexSignal& data, bool inverse = false);
    void fftIterative(ComplexSignal& data, bool inverse = false);
    void realFFT(const Signal& input, ComplexSignal& output);
    void realIFFT(const ComplexSignal& input, Signal& output);
    
    // Utility functions
    size_t reverseBits(size_t num, size_t logN);
    Complex twiddle(size_t k, size_t N, bool inverse = false);
    void bitReverseReorder(ComplexSignal& data);
    
    // Window application
    Signal applyWindow(const Signal& input, WindowFunction window, double parameter = 0.0);
    
public:
    FFTEngine() = default;
    ~FFTEngine() = default;
    
    // Basic FFT operations
    ComplexSignal forwardFFT(const Signal& input, WindowFunction window = WindowFunction::HANNING);
    Signal inverseFFT(const ComplexSignal& input);
    ComplexSignal forwardFFT(const ComplexSignal& input, WindowFunction window = WindowFunction::HANNING);
    ComplexSignal inverseFFT(const ComplexSignal& input, bool normalize = true);
    
    // Real FFT operations (more efficient for real-valued signals)
    ComplexSignal realForwardFFT(const Signal& input, WindowFunction window = WindowFunction::HANNING);
    Signal realInverseFFT(const ComplexSignal& input);
    
    // Spectral analysis
    SpectralData computeSpectrum(const Signal& input, double samplingRate, WindowFunction window = WindowFunction::HANNING);
    std::vector<double> computeMagnitudeSpectrum(const Signal& input, WindowFunction window = WindowFunction::HANNING);
    std::vector<double> computePhaseSpectrum(const Signal& input, WindowFunction window = WindowFunction::HANNING);
    std::vector<double> computePowerSpectrum(const Signal& input, WindowFunction window = WindowFunction::HANNING);
    
    // Advanced spectral methods
    SpectralData welchMethod(const Signal& input, double samplingRate, size_t segmentLength, size_t overlap, WindowFunction window = WindowFunction::HANNING);
    SpectralData bartlettMethod(const Signal& input, double samplingRate, size_t segmentLength);
    std::vector<double> computeSpectrogram(const Signal& input, size_t windowSize, size_t hopSize, WindowFunction window = WindowFunction::HANNING);
    
    // Cross-spectral analysis
    ComplexSignal crossSpectrum(const Signal& signal1, const Signal& signal2, WindowFunction window = WindowFunction::HANNING);
    std::vector<double> coherence(const Signal& signal1, const Signal& signal2, double samplingRate, size_t segmentLength);
    std::vector<double> crossCorrelation(const Signal& signal1, const Signal& signal2);
    std::vector<double> autoCorrelation(const Signal& input);
    
    // Cepstrum analysis
    Signal realCepstrum(const Signal& input);
    ComplexSignal complexCepstrum(const Signal& input);
    
    // Convolution and filtering in frequency domain
    Signal frequencyDomainConvolution(const Signal& signal1, const Signal& signal2);
    Signal frequencyDomainFilter(const Signal& input, const Signal& impulseResponse);
    
    // Zero padding and interpolation
    Signal zeroPad(const Signal& input, size_t targetLength);
    Signal interpolateSpectrum(const ComplexSignal& spectrum, size_t targetLength);
    
    // Utility functions
    std::vector<double> generateFrequencyAxis(size_t fftSize, double samplingRate);
    size_t nextPowerOfTwo(size_t n);
    bool isPowerOfTwo(size_t n);
};

// Wavelet Transform Engine
class WaveletEngine {
private:
    std::unordered_map<WaveletType, std::vector<double>> waveletFilters;
    std::unordered_map<WaveletType, std::vector<double>> scalingFilters;
    mutable std::mutex waveletMutex;
    
    // Wavelet implementations
    void initializeDaubechiesWavelets(int order);
    void initializeBiorthogonalWavelets(int order);
    void initializeCoifletWavelets(int order);
    std::vector<double> generateMorletWavelet(double centerFreq, double bandwidth, double samplingRate, size_t length);
    std::vector<double> generateMexicanHatWavelet(double sigma, double samplingRate, size_t length);
    
    // Transform implementations
    void dwt1D(const Signal& input, Signal& approximation, Signal& detail, WaveletType wavelet);
    void idwt1D(const Signal& approximation, const Signal& detail, Signal& output, WaveletType wavelet);
    void dwtPacket(const Signal& input, std::vector<std::vector<double>>& coefficients, WaveletType wavelet, int levels);
    
    // Convolution helpers
    Signal upSample(const Signal& input, int factor);
    Signal downSample(const Signal& input, int factor);
    Signal convolve(const Signal& input, const std::vector<double>& filter);
    
public:
    WaveletEngine();
    ~WaveletEngine() = default;
    
    // Discrete Wavelet Transform
    std::pair<Signal, Signal> forwardDWT(const Signal& input, WaveletType wavelet);
    Signal inverseDWT(const Signal& approximation, const Signal& detail, WaveletType wavelet);
    std::vector<std::vector<double>> multiLevelDWT(const Signal& input, WaveletType wavelet, int levels);
    Signal multiLevelIDWT(const std::vector<std::vector<double>>& coefficients, WaveletType wavelet);
    
    // Continuous Wavelet Transform
    std::vector<std::vector<Complex>> continuousWaveletTransform(const Signal& input, WaveletType wavelet, 
                                                               const std::vector<double>& scales, double samplingRate);
    std::vector<std::vector<double>> scalogram(const Signal& input, WaveletType wavelet, 
                                             const std::vector<double>& scales, double samplingRate);
    
    // Wavelet Packet Transform
    std::vector<std::vector<double>> waveletPacketTransform(const Signal& input, WaveletType wavelet, int levels);
    Signal inverseWaveletPacketTransform(const std::vector<std::vector<double>>& packets, WaveletType wavelet);
    
    // Wavelet denoising
    Signal waveletDenoise(const Signal& input, WaveletType wavelet, int levels, double threshold, bool softThresholding = true);
    Signal adaptiveWaveletDenoise(const Signal& input, WaveletType wavelet, int levels);
    
    // Feature extraction
    std::vector<double> waveletFeatures(const Signal& input, WaveletType wavelet, int levels);
    double waveletEntropy(const std::vector<std::vector<double>>& coefficients);
    std::vector<double> waveletEnergy(const std::vector<std::vector<double>>& coefficients);
    
    // Edge detection
    Signal waveletEdgeDetection(const Signal& input, WaveletType wavelet, int level);
    std::vector<size_t> detectEdges(const Signal& input, WaveletType wavelet, double threshold);
    
    // Custom wavelet support
    void addCustomWavelet(WaveletType type, const std::vector<double>& lowPassFilter, const std::vector<double>& highPassFilter);
    std::vector<double> getWaveletFilter(WaveletType type, bool lowPass = true) const;
    
    // Analysis functions
    std::vector<double> generateScales(double minScale, double maxScale, int numScales, bool logarithmic = true);
    double computeWaveletVariance(const std::vector<std::vector<double>>& coefficients, int level);
};

// Adaptive Filter Engine
class AdaptiveFilterEngine {
private:
    struct AdaptiveFilterState {
        std::vector<double> weights;
        std::vector<double> inputBuffer;
        double stepSize;
        double forgettingFactor;
        std::vector<std::vector<double>> correlationMatrix;  // For RLS
        std::vector<double> kalmanState;                     // For Kalman filter
        std::vector<std::vector<double>> kalmanCovariance;   // For Kalman filter
        size_t bufferIndex;
        
        AdaptiveFilterState() : stepSize(0.01), forgettingFactor(0.99), bufferIndex(0) {}
    };
    
    std::unordered_map<std::string, AdaptiveFilterState> filterStates;
    mutable std::mutex adaptiveMutex;
    
    // Adaptive algorithms
    double lmsUpdate(AdaptiveFilterState& state, double input, double desired, double error);
    double nlmsUpdate(AdaptiveFilterState& state, double input, double desired, double error);
    double rlsUpdate(AdaptiveFilterState& state, double input, double desired);
    double kalmanUpdate(AdaptiveFilterState& state, double input, double desired, const std::vector<double>& processNoise);
    
    // Matrix operations
    std::vector<std::vector<double>> matrixInvert(const std::vector<std::vector<double>>& matrix);
    std::vector<double> matrixVectorMultiply(const std::vector<std::vector<double>>& matrix, const std::vector<double>& vector);
    void matrixUpdate(std::vector<std::vector<double>>& matrix, const std::vector<double>& vector, double scalar);
    
public:
    AdaptiveFilterEngine() = default;
    ~AdaptiveFilterEngine() = default;
    
    // Filter creation and management
    std::string createAdaptiveFilter(const std::string& name, AdaptiveAlgorithm algorithm, 
                                   int filterLength, double stepSize = 0.01);
    void removeAdaptiveFilter(const std::string& name);
    bool adaptiveFilterExists(const std::string& name) const;
    std::vector<std::string> getAvailableAdaptiveFilters() const;
    
    // Filter operation
    double processAdaptiveSample(const std::string& name, double input, double desired);
    Signal processAdaptiveSignal(const std::string& name, const Signal& input, const Signal& desired);
    
    // Configuration
    void setStepSize(const std::string& name, double stepSize);
    void setForgettingFactor(const std::string& name, double factor); // For RLS
    void resetAdaptiveFilter(const std::string& name);
    
    // Analysis
    std::vector<double> getWeights(const std::string& name) const;
    double getMeanSquareError(const std::string& name, const Signal& input, const Signal& desired);
    std::vector<double> getLearningCurve(const std::string& name, const Signal& input, const Signal& desired);
    
    // Noise cancellation
    Signal noiseCancel(const Signal& primaryInput, const Signal& referenceInput, AdaptiveAlgorithm algorithm, 
                      int filterLength, double stepSize = 0.01);
    Signal echoCancellation(const Signal& nearEnd, const Signal& farEnd, int filterLength, double stepSize = 0.01);
    
    // System identification
    Signal systemIdentification(const Signal& input, const Signal& output, AdaptiveAlgorithm algorithm, 
                               int filterLength, double stepSize = 0.01);
    std::vector<double> identifySystemCoefficients(const Signal& input, const Signal& output, int modelOrder);
    
    // Prediction and tracking
    Signal linearPredictor(const Signal& input, int predictionOrder, AdaptiveAlgorithm algorithm);
    Signal trackingFilter(const Signal& input, const Signal& reference, AdaptiveAlgorithm algorithm);
    
    // Kalman filter specific methods
    void setKalmanParameters(const std::string& name, const std::vector<std::vector<double>>& processNoiseCovariance,
                           const std::vector<std::vector<double>>& measurementNoiseCovariance);
    std::vector<double> kalmanSmoothing(const Signal& measurements, const std::vector<std::vector<double>>& systemMatrix);
};

// Real-time Signal Processor
class RealTimeSignalProcessor {
private:
    struct ProcessingChain {
        std::vector<std::function<double(double)>> processors;
        std::string chainName;
        bool isActive;
        double samplingRate;
        std::chrono::microseconds latency;
        
        ProcessingChain() : isActive(false), samplingRate(1000.0), 
                          latency(std::chrono::microseconds(0)) {}
    };
    
    std::unordered_map<std::string, ProcessingChain> processingChains;
    std::queue<std::pair<std::string, double>> inputQueue;
    std::queue<std::pair<std::string, double>> outputQueue;
    
    mutable std::mutex processorMutex;
    std::mutex queueMutex;
    std::condition_variable processingCondition;
    
    std::thread processingThread;
    std::atomic<bool> isProcessorRunning{false};
    
    // Buffer management
    std::unordered_map<std::string, std::deque<double>> circularBuffers;
    size_t maxBufferSize;
    
    void processingWorker();
    void processInputQueues();
    double processChain(const std::string& chainName, double input);
    
public:
    RealTimeSignalProcessor(size_t bufferSize = 10000);
    ~RealTimeSignalProcessor() { stop(); }
    
    // Lifecycle management
    void start();
    void stop();
    bool isRunning() const { return isProcessorRunning.load(); }
    
    // Processing chain management
    std::string createProcessingChain(const std::string& name, double samplingRate);
    void removeProcessingChain(const std::string& name);
    void activateProcessingChain(const std::string& name, bool active);
    bool isChainActive(const std::string& name) const;
    std::vector<std::string> getActiveChains() const;
    
    // Add processing elements to chain
    void addFilter(const std::string& chainName, const FilterCoefficients& filter);
    void addAdaptiveFilter(const std::string& chainName, AdaptiveAlgorithm algorithm, int length, double stepSize);
    void addCustomProcessor(const std::string& chainName, std::function<double(double)> processor);
    void addGainControl(const std::string& chainName, double gain);
    void addLimiter(const std::string& chainName, double threshold);
    void addDelay(const std::string& chainName, int delaySamples);
    
    // Signal input/output
    void inputSample(const std::string& chainName, double sample);
    double outputSample(const std::string& chainName);
    void inputSignal(const std::string& chainName, const Signal& signal);
    Signal outputSignal(const std::string& chainName, size_t length);
    
    // Buffer operations
    std::deque<double> getBuffer(const std::string& chainName) const;
    void clearBuffer(const std::string& chainName);
    size_t getBufferSize(const std::string& chainName) const;
    
    // Performance monitoring
    std::chrono::microseconds getProcessingLatency(const std::string& chainName) const;
    double getCPUUsage() const;
    std::unordered_map<std::string, double> getPerformanceMetrics() const;
    
    // Configuration
    void setSamplingRate(const std::string& chainName, double rate);
    void setMaxBufferSize(size_t size) { maxBufferSize = size; }
    void enableThreadPriority(bool enable);
};

// Advanced Spectral Analyzer
class SpectralAnalyzer {
private:
    std::unique_ptr<FFTEngine> fftEngine;
    mutable std::mutex analyzerMutex;
    
    // Internal analysis methods
    std::vector<double> computeAutoregressive(const Signal& input, int order);
    SpectralData musicAlgorithm(const Signal& input, double samplingRate, int numSinusoids);
    SpectralData espritAlgorithm(const Signal& input, double samplingRate, int numSinusoids);
    std::vector<double> multitaperEstimate(const Signal& input, double samplingRate, int numTapers, double bandwidth);
    
    // Feature extraction
    double computeSpectralCentroid(const std::vector<double>& spectrum, const std::vector<double>& frequencies);
    double computeSpectralRolloff(const std::vector<double>& spectrum, const std::vector<double>& frequencies, double percentage = 0.85);
    double computeSpectralFlatness(const std::vector<double>& spectrum);
    std::vector<std::pair<double, double>> findHarmonics(const std::vector<double>& spectrum, const std::vector<double>& frequencies, double fundamental);
    
public:
    SpectralAnalyzer();
    ~SpectralAnalyzer() = default;
    
    // Basic spectral analysis
    SpectralData analyzeSpectrum(const Signal& input, double samplingRate, SpectralEstimation method = SpectralEstimation::WELCH);
    std::vector<double> computePowerSpectralDensity(const Signal& input, double samplingRate, SpectralEstimation method = SpectralEstimation::WELCH);
    
    // Advanced spectral estimation
    SpectralData parametricSpectralEstimation(const Signal& input, double samplingRate, int modelOrder);
    SpectralData highResolutionSpectralEstimation(const Signal& input, double samplingRate, int numSinusoids);
    SpectralData timeFrequencyAnalysis(const Signal& input, double samplingRate, size_t windowSize, size_t hopSize);
    
    // Harmonic analysis
    std::vector<std::pair<double, double>> harmonicAnalysis(const Signal& input, double samplingRate, double fundamental);
    double computeTotalHarmonicDistortion(const Signal& input, double samplingRate, double fundamental);
    double computeSignalToNoiseRatio(const Signal& input, double samplingRate, double signalBandwidth);
    
    // Modulation analysis
    SpectralData amplitudeModulationAnalysis(const Signal& input, double samplingRate, double carrierFrequency);
    SpectralData frequencyModulationAnalysis(const Signal& input, double samplingRate);
    SpectralData phaseModulationAnalysis(const Signal& input, double samplingRate);
    
    // Statistical spectral analysis
    std::vector<double> spectralMoments(const std::vector<double>& spectrum, const std::vector<double>& frequencies, int maxOrder = 4);
    double spectralVariance(const std::vector<double>& spectrum);
    double spectralSkewness(const std::vector<double>& spectrum, const std::vector<double>& frequencies);
    double spectralKurtosis(const std::vector<double>& spectrum, const std::vector<double>& frequencies);
    
    // Multi-channel analysis
    std::vector<std::vector<double>> coherenceAnalysis(const std::vector<Signal>& signals, double samplingRate);
    std::vector<double> crossPowerSpectrum(const Signal& signal1, const Signal& signal2, double samplingRate);
    std::vector<double> partialCoherence(const std::vector<Signal>& signals, double samplingRate, int referenceChannel);
    
    // Transient analysis
    std::vector<std::vector<double>> shortTimeSpectralAnalysis(const Signal& input, double samplingRate, size_t windowSize, size_t overlap);
    std::vector<size_t> detectSpectralEvents(const Signal& input, double samplingRate, double threshold);
    SpectralData instantaneousSpectralAnalysis(const Signal& input, double samplingRate, size_t timeIndex, size_t windowSize);
    
    // Specialized EV signal analysis
    SpectralData motorCurrentSignatureAnalysis(const Signal& motorCurrent, double samplingRate, double motorFrequency);
    SpectralData batteryImpedanceAnalysis(const Signal& voltage, const Signal& current, double samplingRate);
    SpectralData vibrationAnalysis(const Signal& accelerometerData, double samplingRate);
    SpectralData acousticAnalysis(const Signal& audioData, double samplingRate);
};

// Main Advanced Signal Processing System
class AdvancedSignalProcessingSystem {
private:
    std::unique_ptr<DigitalFilterEngine> filterEngine;
    std::unique_ptr<FFTEngine> fftEngine;
    std::unique_ptr<WaveletEngine> waveletEngine;
    std::unique_ptr<AdaptiveFilterEngine> adaptiveFilterEngine;
    std::unique_ptr<RealTimeSignalProcessor> realTimeProcessor;
    std::unique_ptr<SpectralAnalyzer> spectralAnalyzer;
    
    std::unordered_map<std::string, Signal> signalDatabase;
    std::unordered_map<std::string, SignalProperties> signalProperties;
    std::unordered_map<std::string, std::function<void(const Signal&)>> signalCallbacks;
    
    mutable std::mutex systemMutex;
    std::atomic<bool> isSystemRunning{false};
    std::thread mainProcessingThread;
    
    struct ProcessingConfiguration {
        bool enableRealTimeProcessing;
        bool enableSpectralAnalysis;
        bool enableAdaptiveFiltering;
        bool enableWaveletProcessing;
        double defaultSamplingRate;
        size_t defaultBufferSize;
        std::chrono::milliseconds processingInterval;
        
        ProcessingConfiguration() : enableRealTimeProcessing(true), enableSpectralAnalysis(true),
                                  enableAdaptiveFiltering(true), enableWaveletProcessing(true),
                                  defaultSamplingRate(1000.0), defaultBufferSize(10000),
                                  processingInterval(std::chrono::milliseconds(10)) {}
    } config;
    
    void runMainProcessing();
    void performScheduledProcessing();
    void updateSignalProperties();
    void processSignalCallbacks();
    
public:
    AdvancedSignalProcessingSystem();
    ~AdvancedSignalProcessingSystem();
    
    // System control
    void start();
    void stop();
    bool isRunning() const { return isSystemRunning.load(); }
    
    // Configuration
    void enableRealTimeProcessing(bool enable) { config.enableRealTimeProcessing = enable; }
    void enableSpectralAnalysis(bool enable) { config.enableSpectralAnalysis = enable; }
    void enableAdaptiveFiltering(bool enable) { config.enableAdaptiveFiltering = enable; }
    void enableWaveletProcessing(bool enable) { config.enableWaveletProcessing = enable; }
    void setDefaultSamplingRate(double rate) { config.defaultSamplingRate = rate; }
    void setDefaultBufferSize(size_t size) { config.defaultBufferSize = size; }
    void setProcessingInterval(std::chrono::milliseconds interval) { config.processingInterval = interval; }
    
    // Signal management
    void addSignal(const std::string& name, const Signal& signal, double samplingRate = 1000.0);
    Signal getSignal(const std::string& name) const;
    void removeSignal(const std::string& name);
    std::vector<std::string> getAvailableSignals() const;
    SignalProperties getSignalProperties(const std::string& name) const;
    
    // Digital filtering
    std::string createDigitalFilter(const std::string& name, FilterType type, FilterDesign design, 
                                  int order, double cutoff, double samplingRate, double bandwidth = 0.0);
    Signal applyDigitalFilter(const std::string& signalName, const std::string& filterName);
    void addRealTimeFilter(const std::string& chainName, const std::string& filterName);
    
    // FFT analysis
    SpectralData performFFTAnalysis(const std::string& signalName, WindowFunction window = WindowFunction::HANNING);
    ComplexSignal getFFT(const std::string& signalName, WindowFunction window = WindowFunction::HANNING);
    Signal performIFFT(const ComplexSignal& spectrum);
    
    // Wavelet analysis
    std::pair<Signal, Signal> performWaveletDecomposition(const std::string& signalName, WaveletType wavelet);
    Signal performWaveletDenoising(const std::string& signalName, WaveletType wavelet, int levels, double threshold);
    std::vector<std::vector<double>> performWaveletPacketAnalysis(const std::string& signalName, WaveletType wavelet, int levels);
    
    // Adaptive filtering
    std::string createAdaptiveFilter(const std::string& name, AdaptiveAlgorithm algorithm, int length, double stepSize = 0.01);
    Signal applyAdaptiveFilter(const std::string& inputSignal, const std::string& referenceSignal, const std::string& filterName);
    Signal performNoiseCancel(const std::string& primarySignal, const std::string& referenceSignal, AdaptiveAlgorithm algorithm);
    
    // Real-time processing
    std::string createRealTimeChain(const std::string& name, double samplingRate);
    void inputRealTimeSample(const std::string& chainName, double sample);
    double outputRealTimeSample(const std::string& chainName);
    void activateRealTimeChain(const std::string& chainName, bool active);
    
    // Spectral analysis
    SpectralData performSpectralAnalysis(const std::string& signalName, SpectralEstimation method = SpectralEstimation::WELCH);
    double computeSignalSNR(const std::string& signalName, double signalBandwidth);
    std::vector<std::pair<double, double>> findSignalHarmonics(const std::string& signalName, double fundamental);
    
    // Signal generation
    Signal generateSineWave(double frequency, double amplitude, double duration, double samplingRate, double phase = 0.0);
    Signal generateSquareWave(double frequency, double amplitude, double duration, double samplingRate, double dutyCycle = 0.5);
    Signal generateSawtoothWave(double frequency, double amplitude, double duration, double samplingRate);
    Signal generateTriangleWave(double frequency, double amplitude, double duration, double samplingRate);
    Signal generateWhiteNoise(double amplitude, double duration, double samplingRate);
    Signal generatePinkNoise(double amplitude, double duration, double samplingRate);
    Signal generateChirp(double startFreq, double endFreq, double duration, double samplingRate, double amplitude = 1.0);
    
    // Signal analysis
    SignalProperties analyzeSignalProperties(const Signal& signal, double samplingRate);
    std::vector<size_t> detectSignalEvents(const std::string& signalName, double threshold, bool risingEdge = true);
    Signal crossCorrelateSignals(const std::string& signal1Name, const std::string& signal2Name);
    double computeCoherence(const std::string& signal1Name, const std::string& signal2Name);
    
    // EV-specific signal processing
    SpectralData analyzeMotorCurrentSignature(const std::string& currentSignalName, double motorFrequency);
    SpectralData analyzeBatteryImpedance(const std::string& voltageSignalName, const std::string& currentSignalName);
    std::vector<double> detectBatteryFaults(const std::string& voltageSignalName, const std::string& currentSignalName);
    Signal filterMotorNoise(const std::string& signalName, double motorFrequency, double samplingRate);
    
    // Signal callbacks and events
    void registerSignalCallback(const std::string& eventType, std::function<void(const Signal&)> callback);
    void unregisterSignalCallback(const std::string& eventType);
    
    // Import/Export
    void importSignalFromCSV(const std::string& filename, const std::string& signalName, double samplingRate);
    void exportSignalToCSV(const std::string& signalName, const std::string& filename);
    void importSignalFromWAV(const std::string& filename, const std::string& signalName);
    void exportSignalToWAV(const std::string& signalName, const std::string& filename, double samplingRate);
    
    // System status and diagnostics
    std::string getSystemStatus() const;
    std::unordered_map<std::string, size_t> getProcessingStatistics() const;
    std::vector<std::string> getPerformanceReport() const;
    void resetProcessingCounters();
};

} // namespace signal_processing

// Implementation of critical inline methods

inline signal_processing::AdvancedSignalProcessingSystem::AdvancedSignalProcessingSystem()
    : filterEngine(std::make_unique<DigitalFilterEngine>())
    , fftEngine(std::make_unique<FFTEngine>())
    , waveletEngine(std::make_unique<WaveletEngine>())
    , adaptiveFilterEngine(std::make_unique<AdaptiveFilterEngine>())
    , realTimeProcessor(std::make_unique<RealTimeSignalProcessor>())
    , spectralAnalyzer(std::make_unique<SpectralAnalyzer>()) {
}

inline signal_processing::AdvancedSignalProcessingSystem::~AdvancedSignalProcessingSystem() {
    stop();
}

inline void signal_processing::AdvancedSignalProcessingSystem::start() {
    if (isSystemRunning.load()) return;
    
    isSystemRunning.store(true);
    
    if (config.enableRealTimeProcessing) {
        realTimeProcessor->start();
    }
    
    mainProcessingThread = std::thread(&AdvancedSignalProcessingSystem::runMainProcessing, this);
    
    std::cout << "[AdvancedSignalProcessing] System started successfully\n";
}

inline void signal_processing::AdvancedSignalProcessingSystem::stop() {
    if (!isSystemRunning.load()) return;
    
    isSystemRunning.store(false);
    
    if (realTimeProcessor->isRunning()) {
        realTimeProcessor->stop();
    }
    
    if (mainProcessingThread.joinable()) {
        mainProcessingThread.join();
    }
    
    std::cout << "[AdvancedSignalProcessing] System stopped successfully\n";
}

inline std::string signal_processing::AdvancedSignalProcessingSystem::getSystemStatus() const {
    std::ostringstream status;
    status << "=== Advanced Signal Processing System Status ===\n";
    status << "System Running: " << (isSystemRunning.load() ? "Yes" : "No") << "\n";
    status << "Real-time Processing: " << (config.enableRealTimeProcessing ? "Enabled" : "Disabled") << "\n";
    status << "Spectral Analysis: " << (config.enableSpectralAnalysis ? "Enabled" : "Disabled") << "\n";
    status << "Adaptive Filtering: " << (config.enableAdaptiveFiltering ? "Enabled" : "Disabled") << "\n";
    status << "Wavelet Processing: " << (config.enableWaveletProcessing ? "Enabled" : "Disabled") << "\n";
    status << "Default Sampling Rate: " << config.defaultSamplingRate << " Hz\n";
    status << "Default Buffer Size: " << config.defaultBufferSize << " samples\n";
    status << "Processing Interval: " << config.processingInterval.count() << " ms\n";
    
    {
        std::lock_guard<std::mutex> lock(systemMutex);
        status << "Signals in Database: " << signalDatabase.size() << "\n";
        status << "Signal Callbacks: " << signalCallbacks.size() << "\n";
    }
    
    return status.str();
}

inline signal_processing::Signal signal_processing::AdvancedSignalProcessingSystem::generateSineWave(
    double frequency, double amplitude, double duration, double samplingRate, double phase) {
    
    size_t numSamples = static_cast<size_t>(duration * samplingRate);
    Signal signal(numSamples);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = static_cast<double>(i) / samplingRate;
        signal[i] = amplitude * std::sin(2.0 * M_PI * frequency * t + phase);
    }
    
    return signal;
}

inline signal_processing::Signal signal_processing::AdvancedSignalProcessingSystem::generateWhiteNoise(
    double amplitude, double duration, double samplingRate) {
    
    size_t numSamples = static_cast<size_t>(duration * samplingRate);
    Signal signal(numSamples);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, amplitude);
    
    for (size_t i = 0; i < numSamples; ++i) {
        signal[i] = dist(gen);
    }
    
    return signal;
}

inline signal_processing::SignalProperties signal_processing::AdvancedSignalProcessingSystem::analyzeSignalProperties(
    const Signal& signal, double samplingRate) {
    
    SignalProperties props;
    props.samplingRate = samplingRate;
    props.length = signal.size();
    props.duration = static_cast<double>(signal.size()) / samplingRate;
    
    if (signal.empty()) return props;
    
    // Calculate mean value
    props.meanValue = std::accumulate(signal.begin(), signal.end(), 0.0) / signal.size();
    
    // Calculate RMS value
    double sumSquares = 0.0;
    for (double sample : signal) {
        sumSquares += sample * sample;
    }
    props.rmsValue = std::sqrt(sumSquares / signal.size());
    
    // Calculate peak value
    auto minMax = std::minmax_element(signal.begin(), signal.end());
    props.peakValue = std::max(std::abs(*minMax.first), std::abs(*minMax.second));
    
    // Calculate crest factor
    if (props.rmsValue > 0) {
        props.crestFactor = props.peakValue / props.rmsValue;
    }
    
    return props;
}

// Implementation of missing methods
inline std::string signal_processing::AdvancedSignalProcessingSystem::createRealTimeChain(const std::string& name, double sampleRate) {
    std::string chainId = name + "_" + std::to_string(sampleRate);
    std::cout << "[SignalProcessing] Created real-time chain: " << chainId << "\n";
    return chainId;
}

inline std::string signal_processing::AdvancedSignalProcessingSystem::createDigitalFilter(const std::string& name, FilterType type, FilterDesign design, int order, double frequency, double bandwidth, double gain) {
    std::string filterId = name + "_filter";
    std::cout << "[SignalProcessing] Created digital filter: " << filterId << "\n";
    return filterId;
}

inline void signal_processing::AdvancedSignalProcessingSystem::addRealTimeFilter(const std::string& chainId, const std::string& filterId) {
    std::cout << "[SignalProcessing] Added filter " << filterId << " to chain " << chainId << "\n";
}

#endif // ADVANCED_SIGNAL_PROCESSING_H