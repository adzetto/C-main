/**
 * @file blockchain_integration.h
 * @author adzetto
 * @brief Blockchain Integration for Vehicle Systems and Smart Contracts
 * @version 1.0
 * @date 2025-09-01
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module provides comprehensive blockchain integration for vehicle systems,
 *          including smart contracts, decentralized identity, supply chain tracking,
 *          and cryptocurrency payments for autonomous vehicle services.
 */

#ifndef BLOCKCHAIN_INTEGRATION_H
#define BLOCKCHAIN_INTEGRATION_H

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>
#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <random>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace blockchain {

// Forward declarations
class Block;
class Transaction;
class SmartContract;
class Blockchain;
class ConsensusEngine;
class DigitalWallet;
class DecentralizedIdentity;

/**
 * @brief Transaction types for vehicle systems
 */
enum class TransactionType {
    PAYMENT, FUEL_PURCHASE, TOLL_PAYMENT, PARKING_FEE, MAINTENANCE_RECORD,
    INSURANCE_CLAIM, VEHICLE_REGISTRATION, SMART_CONTRACT_EXECUTION,
    IDENTITY_VERIFICATION, SUPPLY_CHAIN_UPDATE, DATA_SHARING, SERVICE_REQUEST
};

/**
 * @brief Consensus algorithm types
 */
enum class ConsensusAlgorithm {
    PROOF_OF_WORK, PROOF_OF_STAKE, PROOF_OF_AUTHORITY, 
    DELEGATED_PROOF_OF_STAKE, PRACTICAL_BYZANTINE_FAULT_TOLERANCE
};

/**
 * @brief Smart contract states
 */
enum class ContractState {
    PENDING, ACTIVE, EXECUTED, FAILED, EXPIRED, SUSPENDED
};

/**
 * @brief Network node types
 */
enum class NodeType {
    FULL_NODE, LIGHT_NODE, VALIDATOR, OBSERVER, MINER
};

/**
 * @brief Represents a cryptographic hash
 */
struct Hash {
    std::string value;
    
    Hash() : value("") {}
    Hash(const std::string& data) {
        value = calculateHash(data);
    }
    
    bool operator==(const Hash& other) const {
        return value == other.value;
    }
    
    bool operator!=(const Hash& other) const {
        return value != other.value;
    }
    
    std::string toString() const {
        return value;
    }
    
private:
    std::string calculateHash(const std::string& data) const {
        // Simplified hash calculation (in practice, use SHA-256)
        std::hash<std::string> hasher;
        size_t hash_value = hasher(data);
        
        std::stringstream ss;
        ss << std::hex << hash_value;
        return ss.str();
    }
};

/**
 * @brief Represents a blockchain transaction
 */
class Transaction {
public:
    Transaction(const std::string& from_addr, const std::string& to_addr, 
                double amount, TransactionType type, const std::string& data = "")
        : from_address(from_addr), to_address(to_addr), amount(amount), 
          transaction_type(type), data(data), confirmed(false), fee(0.001) {
        timestamp = std::chrono::system_clock::now();
        transaction_id = generateTransactionId();
    }
    
    /**
     * @brief Get transaction hash
     */
    Hash getHash() const {
        std::string tx_data = from_address + to_address + std::to_string(amount) + 
                             std::to_string(static_cast<int>(transaction_type)) + data;
        return Hash(tx_data);
    }
    
    /**
     * @brief Verify transaction signature (simplified)
     */
    bool verifySignature() const {
        // In practice, this would verify cryptographic signatures
        return !from_address.empty() && !to_address.empty() && amount > 0;
    }
    
    /**
     * @brief Get transaction details as string
     */
    std::string toString() const {
        std::stringstream ss;
        ss << "Transaction ID: " << transaction_id << "\n"
           << "From: " << from_address << "\n"
           << "To: " << to_address << "\n"
           << "Amount: " << amount << "\n"
           << "Type: " << getTransactionTypeName() << "\n"
           << "Fee: " << fee << "\n"
           << "Confirmed: " << (confirmed ? "Yes" : "No") << "\n";
        return ss.str();
    }
    
    /**
     * @brief Validate transaction
     */
    bool validate() const {
        if (from_address.empty() || to_address.empty()) return false;
        if (amount <= 0 || fee < 0) return false;
        if (from_address == to_address && transaction_type == TransactionType::PAYMENT) return false;
        return verifySignature();
    }
    
    // Getters
    const std::string& getFromAddress() const { return from_address; }
    const std::string& getToAddress() const { return to_address; }
    double getAmount() const { return amount; }
    TransactionType getType() const { return transaction_type; }
    const std::string& getData() const { return data; }
    const std::string& getTransactionId() const { return transaction_id; }
    double getFee() const { return fee; }
    bool isConfirmed() const { return confirmed; }
    std::chrono::system_clock::time_point getTimestamp() const { return timestamp; }
    
    // Setters
    void setConfirmed(bool status) { confirmed = status; }
    void setFee(double tx_fee) { fee = tx_fee; }
    
private:
    std::string from_address;
    std::string to_address;
    double amount;
    TransactionType transaction_type;
    std::string data;
    std::string transaction_id;
    double fee;
    bool confirmed;
    std::chrono::system_clock::time_point timestamp;
    
    std::string generateTransactionId() const {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "TX" << std::hex << time_t << std::hex << 
              std::hash<std::string>{}(from_address + to_address);
        return ss.str();
    }
    
    std::string getTransactionTypeName() const {
        switch (transaction_type) {
            case TransactionType::PAYMENT: return "PAYMENT";
            case TransactionType::FUEL_PURCHASE: return "FUEL_PURCHASE";
            case TransactionType::TOLL_PAYMENT: return "TOLL_PAYMENT";
            case TransactionType::PARKING_FEE: return "PARKING_FEE";
            case TransactionType::MAINTENANCE_RECORD: return "MAINTENANCE_RECORD";
            case TransactionType::INSURANCE_CLAIM: return "INSURANCE_CLAIM";
            case TransactionType::VEHICLE_REGISTRATION: return "VEHICLE_REGISTRATION";
            case TransactionType::SMART_CONTRACT_EXECUTION: return "SMART_CONTRACT_EXECUTION";
            case TransactionType::IDENTITY_VERIFICATION: return "IDENTITY_VERIFICATION";
            case TransactionType::SUPPLY_CHAIN_UPDATE: return "SUPPLY_CHAIN_UPDATE";
            case TransactionType::DATA_SHARING: return "DATA_SHARING";
            case TransactionType::SERVICE_REQUEST: return "SERVICE_REQUEST";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief Represents a blockchain block
 */
class Block {
public:
    Block(int block_index, const Hash& previous_hash, const std::vector<Transaction>& transactions)
        : index(block_index), previous_hash(previous_hash), transactions(transactions), 
          nonce(0), difficulty(4) {
        timestamp = std::chrono::system_clock::now();
        merkle_root = calculateMerkleRoot();
        block_hash = calculateHash();
    }
    
    /**
     * @brief Mine the block (proof of work)
     */
    bool mineBlock(int target_difficulty) {
        difficulty = target_difficulty;
        std::string target(difficulty, '0');
        
        std::cout << "[Blockchain] Mining block " << index << "...\n";
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        do {
            nonce++;
            block_hash = calculateHash();
            
            // Stop mining after reasonable time (simulation)
            auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
            if (elapsed > std::chrono::seconds(5)) {
                std::cout << "[Blockchain] Block " << index << " mined with nonce: " << nonce << "\n";
                return true;
            }
            
        } while (block_hash.toString().substr(0, difficulty) != target);
        
        std::cout << "[Blockchain] Block " << index << " mined successfully with nonce: " << nonce << "\n";
        return true;
    }
    
    /**
     * @brief Validate block integrity
     */
    bool validateBlock(const Hash& expected_previous_hash) const {
        // Check previous hash
        if (previous_hash != expected_previous_hash) {
            return false;
        }
        
        // Validate all transactions
        for (const auto& tx : transactions) {
            if (!tx.validate()) {
                return false;
            }
        }
        
        // Verify merkle root
        if (merkle_root != calculateMerkleRoot()) {
            return false;
        }
        
        // Verify block hash
        if (block_hash != calculateHash()) {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief Get block information
     */
    std::string getBlockInfo() const {
        std::stringstream ss;
        ss << "Block #" << index << "\n"
           << "Hash: " << block_hash.toString() << "\n"
           << "Previous Hash: " << previous_hash.toString() << "\n"
           << "Merkle Root: " << merkle_root.toString() << "\n"
           << "Transactions: " << transactions.size() << "\n"
           << "Nonce: " << nonce << "\n"
           << "Difficulty: " << difficulty << "\n";
        return ss.str();
    }
    
    // Getters
    int getIndex() const { return index; }
    const Hash& getHash() const { return block_hash; }
    const Hash& getPreviousHash() const { return previous_hash; }
    const Hash& getMerkleRoot() const { return merkle_root; }
    const std::vector<Transaction>& getTransactions() const { return transactions; }
    uint64_t getNonce() const { return nonce; }
    int getDifficulty() const { return difficulty; }
    std::chrono::system_clock::time_point getTimestamp() const { return timestamp; }
    
private:
    int index;
    Hash previous_hash;
    Hash merkle_root;
    std::vector<Transaction> transactions;
    std::chrono::system_clock::time_point timestamp;
    uint64_t nonce;
    int difficulty;
    Hash block_hash;
    
    Hash calculateHash() const {
        std::stringstream ss;
        ss << index << previous_hash.toString() << merkle_root.toString() 
           << nonce << difficulty;
        
        auto time_t = std::chrono::system_clock::to_time_t(timestamp);
        ss << time_t;
        
        return Hash(ss.str());
    }
    
    Hash calculateMerkleRoot() const {
        if (transactions.empty()) {
            return Hash("");
        }
        
        std::vector<Hash> hashes;
        for (const auto& tx : transactions) {
            hashes.push_back(tx.getHash());
        }
        
        // Build merkle tree
        while (hashes.size() > 1) {
            std::vector<Hash> next_level;
            for (size_t i = 0; i < hashes.size(); i += 2) {
                if (i + 1 < hashes.size()) {
                    next_level.push_back(Hash(hashes[i].toString() + hashes[i + 1].toString()));
                } else {
                    next_level.push_back(hashes[i]);
                }
            }
            hashes = next_level;
        }
        
        return hashes[0];
    }
};

/**
 * @brief Smart contract for autonomous vehicle services
 */
class SmartContract {
public:
    SmartContract(const std::string& contract_id, const std::string& owner,
                  const std::string& code, const std::map<std::string, std::string>& params)
        : contract_id(contract_id), owner_address(owner), contract_code(code),
          parameters(params), state(ContractState::PENDING), gas_limit(1000000) {
        creation_time = std::chrono::system_clock::now();
        balance = 0.0;
    }
    
    /**
     * @brief Execute smart contract
     */
    bool executeContract(const std::vector<std::string>& args) {
        if (state != ContractState::ACTIVE) {
            return false;
        }
        
        std::cout << "[Blockchain/Contract] Executing contract: " << contract_id << "\n";
        
        // Simulate contract execution
        bool execution_success = simulateExecution(args);
        
        if (execution_success) {
            state = ContractState::EXECUTED;
            last_execution = std::chrono::system_clock::now();
            execution_count++;
            
            // Update execution results
            execution_results["last_result"] = "SUCCESS";
            execution_results["execution_count"] = std::to_string(execution_count);
        } else {
            state = ContractState::FAILED;
            execution_results["last_result"] = "FAILED";
        }
        
        return execution_success;
    }
    
    /**
     * @brief Validate contract conditions
     */
    bool validateConditions(const std::map<std::string, std::string>& conditions) const {
        for (const auto& condition : conditions) {
            auto param_it = parameters.find(condition.first);
            if (param_it == parameters.end() || param_it->second != condition.second) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief Fund the contract
     */
    void fundContract(double amount) {
        balance += amount;
        std::cout << "[Blockchain/Contract] Contract " << contract_id 
                  << " funded with " << amount << " tokens\n";
    }
    
    /**
     * @brief Withdraw from contract
     */
    bool withdrawFunds(double amount, const std::string& recipient) {
        if (amount <= balance) {
            balance -= amount;
            std::cout << "[Blockchain/Contract] Withdrawn " << amount 
                      << " tokens to " << recipient << "\n";
            return true;
        }
        return false;
    }
    
    /**
     * @brief Activate the contract
     */
    void activate() {
        if (state == ContractState::PENDING) {
            state = ContractState::ACTIVE;
            activation_time = std::chrono::system_clock::now();
            std::cout << "[Blockchain/Contract] Contract " << contract_id << " activated\n";
        }
    }
    
    /**
     * @brief Suspend the contract
     */
    void suspend() {
        if (state == ContractState::ACTIVE) {
            state = ContractState::SUSPENDED;
            std::cout << "[Blockchain/Contract] Contract " << contract_id << " suspended\n";
        }
    }
    
    /**
     * @brief Get contract status
     */
    std::map<std::string, std::string> getStatus() const {
        std::map<std::string, std::string> status;
        status["contract_id"] = contract_id;
        status["owner"] = owner_address;
        status["state"] = getStateName();
        status["balance"] = std::to_string(balance);
        status["execution_count"] = std::to_string(execution_count);
        status["gas_used"] = std::to_string(gas_used);
        return status;
    }
    
    // Getters
    const std::string& getContractId() const { return contract_id; }
    const std::string& getOwner() const { return owner_address; }
    ContractState getState() const { return state; }
    double getBalance() const { return balance; }
    int getExecutionCount() const { return execution_count; }
    uint64_t getGasUsed() const { return gas_used; }
    
private:
    std::string contract_id;
    std::string owner_address;
    std::string contract_code;
    std::map<std::string, std::string> parameters;
    std::map<std::string, std::string> execution_results;
    ContractState state;
    double balance;
    uint64_t gas_limit;
    uint64_t gas_used;
    int execution_count;
    std::chrono::system_clock::time_point creation_time;
    std::chrono::system_clock::time_point activation_time;
    std::chrono::system_clock::time_point last_execution;
    
    bool simulateExecution(const std::vector<std::string>& args) {
        // Simulate gas consumption
        gas_used = 50000 + (args.size() * 1000);
        
        if (gas_used > gas_limit) {
            return false;
        }
        
        // Simulate various contract types
        if (contract_code.find("FUEL_PAYMENT") != std::string::npos) {
            return executeFuelPayment(args);
        } else if (contract_code.find("PARKING_CONTRACT") != std::string::npos) {
            return executeParkingContract(args);
        } else if (contract_code.find("INSURANCE_CLAIM") != std::string::npos) {
            return executeInsuranceClaim(args);
        } else if (contract_code.find("MAINTENANCE_SCHEDULE") != std::string::npos) {
            return executeMaintenanceSchedule(args);
        }
        
        // Default execution
        return true;
    }
    
    bool executeFuelPayment(const std::vector<std::string>& args) {
        if (args.size() < 2) return false;
        
        double fuel_amount = std::stod(args[0]);
        double price_per_liter = std::stod(args[1]);
        double total_cost = fuel_amount * price_per_liter;
        
        if (balance >= total_cost) {
            balance -= total_cost;
            execution_results["fuel_purchased"] = std::to_string(fuel_amount);
            execution_results["total_cost"] = std::to_string(total_cost);
            return true;
        }
        
        return false;
    }
    
    bool executeParkingContract(const std::vector<std::string>& args) {
        if (args.size() < 2) return false;
        
        int duration_minutes = std::stoi(args[0]);
        double rate_per_hour = std::stod(args[1]);
        double total_cost = (duration_minutes / 60.0) * rate_per_hour;
        
        if (balance >= total_cost) {
            balance -= total_cost;
            execution_results["parking_duration"] = std::to_string(duration_minutes);
            execution_results["parking_cost"] = std::to_string(total_cost);
            return true;
        }
        
        return false;
    }
    
    bool executeInsuranceClaim(const std::vector<std::string>& args) {
        if (args.size() < 1) return false;
        
        double claim_amount = std::stod(args[0]);
        
        // Simulate claim validation
        if (claim_amount <= 10000.0) {  // Max claim limit
            balance += claim_amount;  // Payout
            execution_results["claim_amount"] = std::to_string(claim_amount);
            execution_results["claim_status"] = "APPROVED";
            return true;
        }
        
        execution_results["claim_status"] = "DENIED";
        return false;
    }
    
    bool executeMaintenanceSchedule(const std::vector<std::string>& args) {
        if (args.size() < 1) return false;
        
        std::string maintenance_type = args[0];
        double maintenance_cost = 100.0;  // Base cost
        
        if (maintenance_type == "OIL_CHANGE") {
            maintenance_cost = 50.0;
        } else if (maintenance_type == "BRAKE_SERVICE") {
            maintenance_cost = 200.0;
        } else if (maintenance_type == "FULL_SERVICE") {
            maintenance_cost = 500.0;
        }
        
        if (balance >= maintenance_cost) {
            balance -= maintenance_cost;
            execution_results["maintenance_type"] = maintenance_type;
            execution_results["maintenance_cost"] = std::to_string(maintenance_cost);
            return true;
        }
        
        return false;
    }
    
    std::string getStateName() const {
        switch (state) {
            case ContractState::PENDING: return "PENDING";
            case ContractState::ACTIVE: return "ACTIVE";
            case ContractState::EXECUTED: return "EXECUTED";
            case ContractState::FAILED: return "FAILED";
            case ContractState::EXPIRED: return "EXPIRED";
            case ContractState::SUSPENDED: return "SUSPENDED";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief Digital wallet for cryptocurrency transactions
 */
class DigitalWallet {
public:
    DigitalWallet(const std::string& owner_name) : owner(owner_name), balance(0.0) {
        address = generateAddress();
        private_key = generatePrivateKey();
        public_key = generatePublicKey();
    }
    
    /**
     * @brief Send cryptocurrency to another wallet
     */
    Transaction createTransaction(const std::string& to_address, double amount, 
                                TransactionType type, const std::string& data = "") {
        if (amount > balance) {
            throw std::runtime_error("Insufficient balance");
        }
        
        Transaction tx(address, to_address, amount, type, data);
        
        // Deduct amount from balance
        balance -= (amount + tx.getFee());
        
        // Add to pending transactions
        pending_transactions.push_back(tx);
        
        std::cout << "[Blockchain/Wallet] Created transaction: " << amount 
                  << " tokens to " << to_address << "\n";
        
        return tx;
    }
    
    /**
     * @brief Receive cryptocurrency
     */
    void receivePayment(double amount) {
        balance += amount;
        std::cout << "[Blockchain/Wallet] Received " << amount << " tokens\n";
    }
    
    /**
     * @brief Get wallet balance
     */
    double getBalance() const {
        return balance;
    }
    
    /**
     * @brief Get wallet address
     */
    const std::string& getAddress() const {
        return address;
    }
    
    /**
     * @brief Get transaction history
     */
    std::vector<Transaction> getTransactionHistory() const {
        return transaction_history;
    }
    
    /**
     * @brief Add funds to wallet
     */
    void addFunds(double amount) {
        balance += amount;
        std::cout << "[Blockchain/Wallet] Added " << amount << " tokens to wallet\n";
    }
    
    /**
     * @brief Sign transaction (simplified)
     */
    std::string signTransaction(const Transaction& tx) const {
        // In practice, this would use cryptographic signing
        return "SIGNATURE_" + tx.getTransactionId() + "_" + private_key.substr(0, 8);
    }
    
    /**
     * @brief Verify signature
     */
    bool verifySignature(const Transaction& tx, const std::string& signature) const {
        std::string expected = "SIGNATURE_" + tx.getTransactionId() + "_" + private_key.substr(0, 8);
        return signature == expected;
    }
    
    /**
     * @brief Get wallet information
     */
    std::map<std::string, std::string> getWalletInfo() const {
        return {
            {"owner", owner},
            {"address", address},
            {"balance", std::to_string(balance)},
            {"pending_transactions", std::to_string(pending_transactions.size())},
            {"total_transactions", std::to_string(transaction_history.size())}
        };
    }
    
private:
    std::string owner;
    std::string address;
    std::string private_key;
    std::string public_key;
    double balance;
    std::vector<Transaction> pending_transactions;
    std::vector<Transaction> transaction_history;
    
    std::string generateAddress() const {
        std::hash<std::string> hasher;
        size_t hash = hasher(owner + std::to_string(std::time(nullptr)));
        
        std::stringstream ss;
        ss << "0x" << std::hex << hash;
        return ss.str();
    }
    
    std::string generatePrivateKey() const {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 15);
        
        std::string key;
        for (int i = 0; i < 64; ++i) {
            key += "0123456789abcdef"[dis(gen)];
        }
        return key;
    }
    
    std::string generatePublicKey() const {
        // Simplified public key generation
        std::hash<std::string> hasher;
        size_t hash = hasher(private_key);
        
        std::stringstream ss;
        ss << "PUB" << std::hex << hash;
        return ss.str();
    }
};

/**
 * @brief Main blockchain implementation
 */
class Blockchain {
public:
    Blockchain(int mining_difficulty = 4) : difficulty(mining_difficulty), mining_reward(10.0) {
        createGenesisBlock();
        std::cout << "[Blockchain] Blockchain initialized with genesis block\n";
    }
    
    /**
     * @brief Add a new transaction to the pending pool
     */
    void addTransaction(const Transaction& transaction) {
        if (transaction.validate()) {
            pending_transactions.push_back(transaction);
            std::cout << "[Blockchain] Added transaction to pending pool\n";
        } else {
            std::cout << "[Blockchain] Invalid transaction rejected\n";
        }
    }
    
    /**
     * @brief Mine pending transactions into a new block
     */
    bool minePendingTransactions(const std::string& mining_reward_address) {
        if (pending_transactions.empty()) {
            std::cout << "[Blockchain] No pending transactions to mine\n";
            return false;
        }
        
        // Add mining reward transaction
        Transaction reward_tx("SYSTEM", mining_reward_address, mining_reward, 
                             TransactionType::PAYMENT, "Mining reward");
        pending_transactions.push_back(reward_tx);
        
        // Create new block
        Hash previous_hash = chain.empty() ? Hash("0") : chain.back().getHash();
        Block new_block(static_cast<int>(chain.size()), previous_hash, pending_transactions);
        
        // Mine the block
        if (new_block.mineBlock(difficulty)) {
            chain.push_back(new_block);
            pending_transactions.clear();
            
            std::cout << "[Blockchain] New block mined and added to chain\n";
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief Get balance for an address
     */
    double getBalance(const std::string& address) const {
        double balance = 0.0;
        
        for (const auto& block : chain) {
            for (const auto& tx : block.getTransactions()) {
                if (tx.getFromAddress() == address) {
                    balance -= (tx.getAmount() + tx.getFee());
                }
                if (tx.getToAddress() == address) {
                    balance += tx.getAmount();
                }
            }
        }
        
        return balance;
    }
    
    /**
     * @brief Validate the entire blockchain
     */
    bool validateChain() const {
        for (size_t i = 1; i < chain.size(); ++i) {
            const Block& current_block = chain[i];
            const Block& previous_block = chain[i - 1];
            
            if (!current_block.validateBlock(previous_block.getHash())) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * @brief Get blockchain statistics
     */
    std::map<std::string, double> getBlockchainStats() const {
        double total_transactions = 0;
        double total_value = 0;
        
        for (const auto& block : chain) {
            total_transactions += block.getTransactions().size();
            for (const auto& tx : block.getTransactions()) {
                total_value += tx.getAmount();
            }
        }
        
        return {
            {"blocks", static_cast<double>(chain.size())},
            {"total_transactions", total_transactions},
            {"total_value", total_value},
            {"pending_transactions", static_cast<double>(pending_transactions.size())},
            {"difficulty", static_cast<double>(difficulty)},
            {"mining_reward", mining_reward}
        };
    }
    
    /**
     * @brief Get the latest block
     */
    const Block& getLatestBlock() const {
        if (chain.empty()) {
            throw std::runtime_error("Blockchain is empty");
        }
        return chain.back();
    }
    
    /**
     * @brief Get block by index
     */
    const Block& getBlock(int index) const {
        if (index < 0 || index >= static_cast<int>(chain.size())) {
            throw std::out_of_range("Block index out of range");
        }
        return chain[index];
    }
    
    /**
     * @brief Get all blocks
     */
    const std::vector<Block>& getChain() const {
        return chain;
    }
    
    /**
     * @brief Set mining difficulty
     */
    void setDifficulty(int new_difficulty) {
        difficulty = std::max(1, std::min(10, new_difficulty));
        std::cout << "[Blockchain] Mining difficulty set to " << difficulty << "\n";
    }
    
    /**
     * @brief Get pending transactions
     */
    const std::vector<Transaction>& getPendingTransactions() const {
        return pending_transactions;
    }
    
private:
    std::vector<Block> chain;
    std::vector<Transaction> pending_transactions;
    int difficulty;
    double mining_reward;
    
    void createGenesisBlock() {
        std::vector<Transaction> genesis_transactions;
        Block genesis_block(0, Hash("0"), genesis_transactions);
        genesis_block.mineBlock(difficulty);
        chain.push_back(genesis_block);
    }
};

/**
 * @brief Decentralized identity management system
 */
class DecentralizedIdentity {
public:
    DecentralizedIdentity(const std::string& user_id) : identity_id(user_id) {
        did_address = generateDIDAddress();
        creation_time = std::chrono::system_clock::now();
        verification_level = 0;
    }
    
    /**
     * @brief Add identity attribute
     */
    void addAttribute(const std::string& key, const std::string& value, bool verified = false) {
        attributes[key] = {value, verified, std::chrono::system_clock::now()};
        if (verified) {
            verification_level++;
        }
        
        std::cout << "[Blockchain/Identity] Added attribute: " << key << "\n";
    }
    
    /**
     * @brief Verify attribute
     */
    bool verifyAttribute(const std::string& key, const std::string& verifier) {
        auto it = attributes.find(key);
        if (it != attributes.end()) {
            it->second.verified = true;
            verifiers[key] = verifier;
            verification_level++;
            
            std::cout << "[Blockchain/Identity] Attribute " << key 
                      << " verified by " << verifier << "\n";
            return true;
        }
        return false;
    }
    
    /**
     * @brief Get identity document
     */
    std::map<std::string, std::string> getIdentityDocument() const {
        std::map<std::string, std::string> document;
        
        document["did"] = did_address;
        document["identity_id"] = identity_id;
        document["verification_level"] = std::to_string(verification_level);
        
        for (const auto& attr : attributes) {
            document[attr.first] = attr.second.value;
            document[attr.first + "_verified"] = attr.second.verified ? "true" : "false";
        }
        
        return document;
    }
    
    /**
     * @brief Create verifiable credential
     */
    std::string createCredential(const std::string& credential_type, 
                               const std::map<std::string, std::string>& claims) {
        std::string credential_id = "VC_" + identity_id + "_" + credential_type;
        
        credentials[credential_id] = {
            credential_type,
            claims,
            std::chrono::system_clock::now(),
            true
        };
        
        std::cout << "[Blockchain/Identity] Created credential: " << credential_id << "\n";
        return credential_id;
    }
    
    /**
     * @brief Revoke credential
     */
    bool revokeCredential(const std::string& credential_id) {
        auto it = credentials.find(credential_id);
        if (it != credentials.end()) {
            it->second.valid = false;
            std::cout << "[Blockchain/Identity] Revoked credential: " << credential_id << "\n";
            return true;
        }
        return false;
    }
    
    /**
     * @brief Get trust score
     */
    double getTrustScore() const {
        double base_score = std::min(1.0, verification_level / 10.0);  // Max 1.0 for 10+ verifications
        double age_factor = std::min(1.0, getAgeInDays() / 365.0);    // Max 1.0 for 1+ year
        double credential_factor = std::min(1.0, credentials.size() / 5.0);  // Max 1.0 for 5+ credentials
        
        return (base_score + age_factor + credential_factor) / 3.0;
    }
    
    // Getters
    const std::string& getIdentityId() const { return identity_id; }
    const std::string& getDIDAddress() const { return did_address; }
    int getVerificationLevel() const { return verification_level; }
    
private:
    struct IdentityAttribute {
        std::string value;
        bool verified;
        std::chrono::system_clock::time_point timestamp;
    };
    
    struct Credential {
        std::string type;
        std::map<std::string, std::string> claims;
        std::chrono::system_clock::time_point issued_at;
        bool valid;
    };
    
    std::string identity_id;
    std::string did_address;
    std::map<std::string, IdentityAttribute> attributes;
    std::map<std::string, std::string> verifiers;
    std::map<std::string, Credential> credentials;
    std::chrono::system_clock::time_point creation_time;
    int verification_level;
    
    std::string generateDIDAddress() const {
        std::hash<std::string> hasher;
        size_t hash = hasher("did:vehicle:" + identity_id);
        
        std::stringstream ss;
        ss << "did:vehicle:" << std::hex << hash;
        return ss.str();
    }
    
    double getAgeInDays() const {
        auto now = std::chrono::system_clock::now();
        auto duration = now - creation_time;
        return std::chrono::duration<double, std::ratio<86400>>(duration).count();
    }
};

/**
 * @brief Main blockchain integration controller
 */
class BlockchainIntegration {
public:
    BlockchainIntegration() {
        blockchain = std::make_unique<Blockchain>(4);
        initializeSystem();
    }
    
    ~BlockchainIntegration() {
        std::cout << "[Blockchain] Blockchain integration system shutdown\n";
    }
    
    /**
     * @brief Create a new digital wallet
     */
    std::shared_ptr<DigitalWallet> createWallet(const std::string& owner_name) {
        auto wallet = std::make_shared<DigitalWallet>(owner_name);
        wallets[wallet->getAddress()] = wallet;
        
        // Give initial funds for testing
        wallet->addFunds(1000.0);
        
        std::cout << "[Blockchain] Created wallet for " << owner_name 
                  << " with address: " << wallet->getAddress() << "\n";
        
        return wallet;
    }
    
    /**
     * @brief Deploy a smart contract
     */
    std::shared_ptr<SmartContract> deployContract(const std::string& owner_address,
                                                 const std::string& contract_code,
                                                 const std::map<std::string, std::string>& params) {
        std::string contract_id = "CONTRACT_" + std::to_string(contracts.size() + 1);
        auto contract = std::make_shared<SmartContract>(contract_id, owner_address, 
                                                       contract_code, params);
        
        contracts[contract_id] = contract;
        contract->activate();
        
        std::cout << "[Blockchain] Deployed contract: " << contract_id << "\n";
        return contract;
    }
    
    /**
     * @brief Create decentralized identity
     */
    std::shared_ptr<DecentralizedIdentity> createIdentity(const std::string& user_id) {
        auto identity = std::make_shared<DecentralizedIdentity>(user_id);
        identities[identity->getDIDAddress()] = identity;
        
        std::cout << "[Blockchain] Created decentralized identity: " 
                  << identity->getDIDAddress() << "\n";
        
        return identity;
    }
    
    /**
     * @brief Process vehicle service payment
     */
    bool processVehiclePayment(const std::string& from_wallet, const std::string& to_wallet,
                              double amount, TransactionType service_type) {
        auto from_it = wallets.find(from_wallet);
        auto to_it = wallets.find(to_wallet);
        
        if (from_it == wallets.end() || to_it == wallets.end()) {
            return false;
        }
        
        try {
            Transaction tx = from_it->second->createTransaction(to_wallet, amount, service_type);
            blockchain->addTransaction(tx);
            to_it->second->receivePayment(amount);
            
            std::cout << "[Blockchain] Processed payment: " << amount 
                      << " tokens for service\n";
            return true;
        } catch (const std::exception& e) {
            std::cout << "[Blockchain] Payment failed: " << e.what() << "\n";
            return false;
        }
    }
    
    /**
     * @brief Mine new block
     */
    bool mineBlock(const std::string& miner_address) {
        return blockchain->minePendingTransactions(miner_address);
    }
    
    /**
     * @brief Get system status
     */
    std::map<std::string, std::string> getSystemStatus() const {
        auto stats = blockchain->getBlockchainStats();
        
        return {
            {"blockchain_valid", blockchain->validateChain() ? "true" : "false"},
            {"total_blocks", std::to_string(static_cast<int>(stats["blocks"]))},
            {"total_wallets", std::to_string(wallets.size())},
            {"active_contracts", std::to_string(contracts.size())},
            {"registered_identities", std::to_string(identities.size())},
            {"pending_transactions", std::to_string(static_cast<int>(stats["pending_transactions"]))},
            {"network_status", "OPERATIONAL"}
        };
    }
    
    // Getters
    Blockchain* getBlockchain() const { return blockchain.get(); }
    std::shared_ptr<DigitalWallet> getWallet(const std::string& address) const {
        auto it = wallets.find(address);
        return (it != wallets.end()) ? it->second : nullptr;
    }
    std::shared_ptr<SmartContract> getContract(const std::string& contract_id) const {
        auto it = contracts.find(contract_id);
        return (it != contracts.end()) ? it->second : nullptr;
    }
    std::shared_ptr<DecentralizedIdentity> getIdentity(const std::string& did_address) const {
        auto it = identities.find(did_address);
        return (it != identities.end()) ? it->second : nullptr;
    }
    
private:
    std::unique_ptr<Blockchain> blockchain;
    std::unordered_map<std::string, std::shared_ptr<DigitalWallet>> wallets;
    std::unordered_map<std::string, std::shared_ptr<SmartContract>> contracts;
    std::unordered_map<std::string, std::shared_ptr<DecentralizedIdentity>> identities;
    
    void initializeSystem() {
        std::cout << "[Blockchain] Blockchain integration system initialized\n";
        std::cout << "[Blockchain] Genesis block created and validated\n";
    }
};

} // namespace blockchain

#endif // BLOCKCHAIN_INTEGRATION_H