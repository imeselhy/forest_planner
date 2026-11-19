// path_receiver_node.cpp
// ROS2 Humble — r2_navigation package
//
// 1. HTTP server on port 8080 receives PWA POST /plan (JSON)
// 2. Parses + validates JSON → builds compact R2Path message
// 3. Publishes R2Path to /r2_path
// 4. Packs path into CAN frames using can_packet.hpp
// 5. Logs CAN frames (wire these to your CAN peripheral / SocketCAN)

#include <rclcpp/rclcpp.hpp>
#include "r2_navigation/msg/r2_path.hpp"
#include "r2_navigation/msg/r2_step.hpp"
#include "r2_navigation/can_packet.hpp"

#include "httplib.h"
#include "json.hpp"

#include <thread>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <cstring>

using json   = nlohmann::json;
using R2Path = r2_navigation::msg::R2Path;
using R2Step = r2_navigation::msg::R2Step;

// ── Block height table (mm, 1-indexed) ─────────────────────────────────────
// B1=400 B2=200 B3=400 B4=200 B5=400 B6=600
// B7=400 B8=600 B9=400 B10=200 B11=400 B12=200
static constexpr uint16_t BLOCK_HEIGHT_MM[13] = {
    0,                              // index 0 unused
    400, 200, 400, 200, 400, 600,   // B1–B6
    400, 600, 400, 200, 400, 200    // B7–B12
};

// height_delta_enc: -400=0, -200=1, 0=2, +200=3, +400=4
static uint8_t heightDeltaEnc(uint8_t cur, uint8_t tgt) {
    if (cur < 1 || cur > 12 || tgt < 1 || tgt > 12) return 2; // 0mm fallback
    int16_t d = (int16_t)BLOCK_HEIGHT_MM[tgt] - (int16_t)BLOCK_HEIGHT_MM[cur];
    switch (d) {
        case -400: return 0;
        case -200: return 1;
        case    0: return 2;
        case  200: return 3;
        case  400: return 4;
        default:   return 2;
    }
}
// PRE_ENTRY: robot is on the ground (0mm), delta = absolute target block height.
// 200mm→enc3, 400mm→enc4, 600mm→enc5 (enc5 reserved slot, always positive lift)
static uint8_t preEntryHeightDeltaEnc(uint8_t tgt) {
    if (tgt < 1 || tgt > 12) return 2;
    switch (BLOCK_HEIGHT_MM[tgt]) {
        case 200: return 3;  // +200mm
        case 400: return 4;  // +400mm
        case 600: return 5;  // +600mm
        default:  return 2;
    }
}

class PathReceiverNode : public rclcpp::Node
{
public:
    PathReceiverNode() : Node("path_receiver")
    {
        this->declare_parameter<int>("http_port",           8080);
        this->declare_parameter<std::string>("http_host",   "0.0.0.0");
        this->declare_parameter<std::string>("path_topic",  "/r2_path");

        int         port  = this->get_parameter("http_port").as_int();
        std::string host  = this->get_parameter("http_host").as_string();
        std::string topic = this->get_parameter("path_topic").as_string();

        publisher_ = this->create_publisher<R2Path>(topic, 10);

        RCLCPP_INFO(get_logger(), "Path receiver node started");
        RCLCPP_INFO(get_logger(), "Publishing to : %s", topic.c_str());
        RCLCPP_INFO(get_logger(), "HTTP endpoint : http://%s:%d/plan", host.c_str(), port);

        server_thread_ = std::thread([this, host, port](){
            startHttpServer(host, port);
        });
    }

    ~PathReceiverNode() override {
        server_.stop();
        if (server_thread_.joinable()) server_thread_.join();
    }

private:
    rclcpp::Publisher<R2Path>::SharedPtr publisher_;
    httplib::Server server_;
    std::thread     server_thread_;

    void startHttpServer(const std::string& host, int port)
    {
        server_.Options("/plan", [](const httplib::Request&, httplib::Response& res){
            res.set_header("Access-Control-Allow-Origin",  "*");
            res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type");
            res.status = 204;
        });

        server_.Post("/plan", [this](const httplib::Request& req, httplib::Response& res){
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Content-Type", "application/json");
            RCLCPP_INFO(get_logger(), "POST /plan (%zu bytes)", req.body.size());
            try {
                auto result = parsePayload(req.body);
                publisher_->publish(result.first);
                RCLCPP_INFO(get_logger(),
                    "Published /r2_path  entry=B%d  exit=B%d  cost=%d  steps=%zu",
                    result.first.entry_block, result.first.exit_block,
                    result.first.total_cost,  result.first.steps.size());
                r2can::CanFrames frames = r2can::pack(result.second);
                logCanFrames(frames);
                res.status = 200;
                res.set_content(
                    R"({"status":"ok","message":"Path received and published"})",
                    "application/json");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Parse error: %s", e.what());
                res.status = 400;
                res.set_content(
                    std::string(R"({"status":"error","message":")") + e.what() + "\"}",
                    "application/json");
            }
        });

        server_.Get("/health", [](const httplib::Request&, httplib::Response& res){
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(R"({"status":"ok","node":"path_receiver"})",
                            "application/json");
        });

        if (!server_.listen(host.c_str(), port))
            RCLCPP_FATAL(get_logger(), "HTTP server failed on %s:%d",
                         host.c_str(), port);
    }

    // Returns {ROS2 message, CAN packet}
    std::pair<R2Path, r2can::PathPacket> parsePayload(const std::string& body)
    {
        json j = json::parse(body);

        R2Path            ros_msg;
        r2can::PathPacket can_pkt;
        memset(&can_pkt, 0, sizeof(can_pkt));

        ros_msg.header.stamp    = get_clock()->now();
        ros_msg.header.frame_id = "forest_grid";

        ros_msg.total_cost       = getReq<uint8_t>(j, "total_cost");
        ros_msg.entry_block      = getReq<uint8_t>(j, "entry_block");
        ros_msg.exit_block       = getReq<uint8_t>(j, "exit_block");
        ros_msg.kfs_total        = j.value("kfs_total",        3);
        ros_msg.kfs_stored       = j.value("kfs_stored",       0);
        ros_msg.boxes_to_collect = ros_msg.kfs_total - ros_msg.kfs_stored;
        ros_msg.retry            = j.value("retry",            false);

        can_pkt.total_cost  = ros_msg.total_cost;
        can_pkt.entry_block = ros_msg.entry_block;
        can_pkt.exit_block  = ros_msg.exit_block;
        can_pkt.kfs_total   = ros_msg.kfs_total;
        can_pkt.kfs_stored  = ros_msg.kfs_stored;
        can_pkt.retry       = ros_msg.retry ? 1u : 0u;

        if (ros_msg.entry_block < 1 || ros_msg.entry_block > 3)
            throw std::runtime_error("entry_block must be 1-3");
        if (ros_msg.exit_block < 10 || ros_msg.exit_block > 12)
            throw std::runtime_error("exit_block must be 10-12");

        // Grid state
        if (!j.contains("grid_state") || !j["grid_state"].is_array()
            || j["grid_state"].size() != 12)
            throw std::runtime_error("grid_state must be array[12]");

        for (int i = 0; i < 12; i++) {
            uint8_t v = j["grid_state"][i].get<uint8_t>();
            if (v > 3) throw std::runtime_error("grid_state[" + std::to_string(i) + "] out of range");
            // FAKE box not allowed on entry blocks B1, B2, B3 (indices 0,1,2)
            if (v == 3 && i < 3)
                throw std::runtime_error("FAKE box not allowed on entry block B" + std::to_string(i+1));
            ros_msg.grid_state[i] = v;
            can_pkt.grid_state[i] = v;
        }

        // Pre-entry pickups
        if (j.contains("pre_entry_pickups") && j["pre_entry_pickups"].is_array()) {
            for (auto& b : j["pre_entry_pickups"])
                ros_msg.pre_entry_pickups.push_back(b.get<uint8_t>());
        }
        can_pkt.pre_entry_count = (uint8_t)ros_msg.pre_entry_pickups.size();

        // Steps
        if (!j.contains("steps") || !j["steps"].is_array() || j["steps"].empty())
            throw std::runtime_error("steps array missing or empty");

        uint8_t sidx = 0;
        for (auto& s : j["steps"]) {
            R2Step rs;
            rs.step_index        = sidx;
            rs.action_type       = getReq<uint8_t>(s, "action_type");
            rs.current_block     = getReq<uint8_t>(s, "current_block");
            rs.target_block      = getReq<uint8_t>(s, "target_block");
            rs.direction         = getReq<uint8_t>(s, "direction");
            rs.collected_after   = getReq<uint8_t>(s, "collected_after");
            rs.requires_r1_clear = s.value("requires_r1_clear", false);
            rs.auto_pickup       = s.value("auto_pickup",        false);
            // height delta — computed server-side from block height table
            rs.height_delta_enc  = (rs.current_block == 0)
                ? preEntryHeightDeltaEnc(rs.target_block)
                : heightDeltaEnc(rs.current_block, rs.target_block);
            rs.height_delta_mm   = [](uint8_t enc) -> int16_t {
                switch(enc){ case 0:return -400; case 1:return -200; case 2:return 0;
                             case 3:return  200; case 4:return  400; case 5:return 600;
                             default: return 0; }
            }(rs.height_delta_enc);

            if (rs.action_type > 2)
                throw std::runtime_error("action_type out of range at step " + std::to_string(sidx));
            if (rs.direction > 3)
                throw std::runtime_error("direction out of range at step " + std::to_string(sidx));
            // current_block==0 is valid: means robot is outside the grid (PRE_ENTRY or ENTER step)
            if (rs.current_block > 12 || rs.target_block < 1 || rs.target_block > 12)
                throw std::runtime_error("block out of range at step " + std::to_string(sidx));

            ros_msg.steps.push_back(rs);

            if (sidx < r2can::MAX_STEPS) {
                r2can::Step& cs      = can_pkt.steps[sidx];
                cs.action_type       = rs.action_type;
                cs.current_block     = rs.current_block;
                cs.target_block      = rs.target_block;
                cs.direction         = rs.direction;
                cs.collected_after   = rs.collected_after;
                cs.requires_r1_clear = rs.requires_r1_clear;
                cs.auto_pickup       = rs.auto_pickup;
                // current_block==0 means robot is on the ground (PRE_ENTRY or ENTER step).
                cs.height_delta_enc = (rs.current_block == 0)
                    ? preEntryHeightDeltaEnc(rs.target_block)
                    : heightDeltaEnc(rs.current_block, rs.target_block);
                can_pkt.step_count++;
            }
            sidx++;
        }

        return { ros_msg, can_pkt };
    }

    void logCanFrames(const r2can::CanFrames& cf)
    {
        RCLCPP_INFO(get_logger(), "CAN packet ready: %d frame(s), %d bytes on bus",
                    cf.count, cf.count * r2can::CAN_FRAME_BYTES);
        for (uint8_t i = 0; i < cf.count; i++) {
            std::ostringstream ss;
            ss << "  Frame[" << (int)i << "]: ";
            for (int b = 0; b < r2can::CAN_FRAME_BYTES; b++)
                ss << std::hex << std::uppercase << std::setw(2)
                   << std::setfill('0') << (int)cf.data[i][b] << " ";
            ss << " CRC=0x" << std::hex << std::uppercase
               << std::setw(2) << std::setfill('0') << (int)cf.data[i][7];
            RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        }
    }

    template<typename T>
    T getReq(const json& j, const std::string& key) {
        if (!j.contains(key))
            throw std::runtime_error("Missing field: " + key);
        return j[key].get<T>();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
