/**
 * Cabin AI Assistant (Intent + Dialogue)
 * Author: adzetto
 */
#ifndef CABIN_ASSISTANT_H
#define CABIN_ASSISTANT_H

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <iostream>

namespace assistant
{
    struct Context { std::unordered_map<std::string,std::string> slots; };
    struct Intent { std::string name; std::unordered_map<std::string,std::string> entities; };

    class NLU
    {
    public:
        Intent parse(const std::string& utterance) const {
            // Very simple rules
            if (utterance.find("temperature") != std::string::npos) return {"set_temperature", {}};
            if (utterance.find("navigate") != std::string::npos) return {"navigate", {}};
            if (utterance.find("music") != std::string::npos) return {"play_music", {}};
            return {"unknown", {}};
        }
    };

    class DialogueManager
    {
    public:
        void on_intent(const Intent& i) {
            history_.push_back(i.name);
            auto it = handlers_.find(i.name);
            if (it != handlers_.end()) it->second(i);
            else std::cout << "Assistant: I didn't catch that.\n";
        }
        void add_handler(const std::string& name, std::function<void(const Intent&)> fn) { handlers_[name]=std::move(fn); }
        void set_context(const Context& c) { ctx_ = c; }
        const std::vector<std::string>& history() const { return history_; }
    private:
        Context ctx_{};
        std::unordered_map<std::string,std::function<void(const Intent&)>> handlers_{};
        std::vector<std::string> history_{};
    };

    class CabinAssistant
    {
    public:
        CabinAssistant() {
            dm_.add_handler("set_temperature", [&](const Intent&){ std::cout << "Setting cabin temp.\n"; });
            dm_.add_handler("navigate", [&](const Intent&){ std::cout << "Starting navigation.\n"; });
            dm_.add_handler("play_music", [&](const Intent&){ std::cout << "Playing music.\n"; });
        }
        void handle_utterance(const std::string& u) {
            auto intent = nlu_.parse(u); dm_.on_intent(intent);
        }
        void set_context(const Context& c) { dm_.set_context(c); }
    private:
        NLU nlu_{}; DialogueManager dm_{};
    };
}

#endif // CABIN_ASSISTANT_H

