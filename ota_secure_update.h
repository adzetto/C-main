/**
 * Secure OTA Update Manager
 * Author: adzetto
 */
#ifndef OTA_SECURE_UPDATE_H
#define OTA_SECURE_UPDATE_H

#include <string>
#include <vector>
#include <optional>
#include <iostream>

namespace ota
{
    struct BundleMeta {
        std::string id;
        std::string version;
        std::string hash;
        size_t size_bytes{0};
        bool delta{false};
    };

    enum class VerifyResult { OK, HASH_MISMATCH, SIGNATURE_INVALID, CORRUPT };
    enum class ApplyResult { APPLIED, ROLLED_BACK, FAILED };

    class Verifier
    {
    public:
        VerifyResult verify(const BundleMeta& meta, const std::string& /*path*/) const {
            if (meta.hash.empty()) return VerifyResult::HASH_MISMATCH; // stub
            return VerifyResult::OK;
        }
    };

    class Installer
    {
    public:
        ApplyResult apply(const BundleMeta& meta) {
            (void)meta; // pretend success
            last_applied_ = meta;
            return ApplyResult::APPLIED;
        }
        ApplyResult rollback() {
            if (!last_applied_.has_value()) return ApplyResult::FAILED;
            rolled_back_ = last_applied_; last_applied_.reset();
            return ApplyResult::ROLLED_BACK;
        }
        std::optional<BundleMeta> last_applied() const { return last_applied_; }
    private:
        std::optional<BundleMeta> last_applied_{};
        std::optional<BundleMeta> rolled_back_{};
    };

    class UpdateManager
    {
    public:
        VerifyResult verify(const BundleMeta& m, const std::string& path) { return verifier_.verify(m, path); }
        ApplyResult install(const BundleMeta& m) { return installer_.apply(m); }
        ApplyResult rollback() { return installer_.rollback(); }
        void set_current_version(const std::string& v) { current_version_ = v; }
        std::string current_version() const { return current_version_; }
        void display_status() const {
            std::cout << "\n=== OTA Update Status ===\n";
            std::cout << "Current: " << current_version_ << "\n";
            auto last = installer_.last_applied();
            if (last) std::cout << "Last applied: " << last->version << " (" << last->id << ")\n";
        }
    private:
        std::string current_version_{"1.0.0"};
        Verifier verifier_{};
        Installer installer_{};
    };
}

#endif // OTA_SECURE_UPDATE_H

