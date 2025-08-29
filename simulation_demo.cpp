// Simulation Demo integrating new subsystems behind simple scenarios
// Author: adzetto

#include <iostream>
#include <vector>

#include "iso15118_ccs.h"
#include "v2g_grid_integration.h"
#include "route_mapping.h"
#include "localization_sensors.h"
#include "fleet_telematics.h"
#include "predictive_maintenance.h"
#include "functional_safety.h"
#include "ota_secure_update.h"
#include "cabin_assistant.h"
#include "security_monitoring.h"
#include "dc_fast_charging_control.h"
#include "simulation_toolkit.h"

int main() {
    std::cout << "Simulation Demo Start\n";

    // ISO 15118 session
    iso15118::ISO15118Stack iso;
    iso.connect();
    iso.start(iso15118::EVRequest{}, iso15118::EVSEOffer{});
    iso.authenticate();
    auto mi = iso.deliver(50.0);
    std::cout << "ISO15118 deliver: " << mi.power << " kW\n";

    // V2G decision
    v2g::V2GController v2gctl("VIN123", {11,7.4,20,90});
    auto cmd = v2gctl.step({49.85, 55.0, 300}, 70);
    std::cout << "V2G cmd power: " << cmd.target_power_kw << " kW\n";

    // Routing
    routing::Graph g; int a=g.add_node(0,0), b=g.add_node(10,0), c=g.add_node(10,10);
    g.add_edge(a,b,10,true); g.add_edge(b,c,10,true);
    routing::AStar astar; auto path = astar.find_path(g,a,c);
    std::cout << "Path nodes: " << path.size() << "\n";

    // Localization
    localization::EKF ekf; ekf.predict(0.1, {0,0,0.01}); ekf.update_gps({5,0,1,1,true});
    auto pose = ekf.state(); std::cout << "Pose x=" << pose.x << " y=" << pose.y << "\n";

    // Telematics
    telematics::DataRecorder rec; rec.begin_trip("T001"); rec.add(37.0, -122.0, 60.0, 75.0); auto trip = rec.end_trip();
    std::cout << "Trip samples: " << trip.samples.size() << "\n";

    // Predictive maintenance
    pm::MaintenanceAdvisor advisor; advisor.add_component("motor"); advisor.add_component("battery");
    auto health = advisor.assess({pm::FeatureVector{{0.1,0.2}}, pm::FeatureVector{{0.3}}});
    std::cout << "Health components: " << health.size() << "\n";

    // Safety monitor
    safety::SafetyMonitor sm; sm.log_event({std::chrono::system_clock::now(), safety::FaultType::POWER, safety::HazardLevel::ASIL_B, "inverter", "overcurrent"});
    sm.apply_mitigations(); sm.display_status();

    // OTA update
    ota::UpdateManager um; um.set_current_version("1.2.3"); um.verify({"id","1.2.4","hash",1024,false}, "bundle.bin"); um.install({"id","1.2.4","hash",1024,false}); um.display_status();

    // Cabin assistant
    assistant::CabinAssistant ca; ca.handle_utterance("set temperature to 20");

    // Security monitor
    secmon::SecurityMonitor mon; mon.add_rule("CAN_REPLAY", secmon::Severity::ALERT); mon.observe("canbus","CAN_REPLAY"); mon.report();

    // DC fast charging control
    dcfc::Controller dc; dc.set_limits({1000,500,350}); auto sp = dc.track_power(120.0, 400.0, 100.0, 0.1); (void)sp;

    // Simulation toolkit
    sim::Simulator sim;
    sim.add({"simple", {{0.0, [](){ std::cout << "Step 0\n"; }}, {1.0, [](){ std::cout << "Step 1\n"; }}}});
    sim.run_all();

    std::cout << "Simulation Demo End\n";
    return 0;
}

