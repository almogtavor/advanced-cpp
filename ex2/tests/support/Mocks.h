#pragma once

// GMock doubles for every interface, so each component can be tested in
// isolation from its collaborators.

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>

#include <gmock/gmock.h>

namespace dmtest {

namespace dm = drone_mapper;
namespace t = drone_mapper::types;

class MockLidar : public dm::ILidar {
public:
    MOCK_METHOD(t::LidarScanResult, scan, (dm::Orientation), (const, override));
    MOCK_METHOD(t::LidarConfigData, config, (), (const, override));
};

class MockGps : public dm::IGPS {
public:
    MOCK_METHOD(dm::Position3D, position, (), (const, override));
    MOCK_METHOD(dm::Orientation, heading, (), (const, override));
};

class MockMovement : public dm::IDroneMovement {
public:
    MOCK_METHOD(t::MovementResult, rotate, (t::RotationDirection, dm::HorizontalAngle), (override));
    MOCK_METHOD(t::MovementResult, advance, (dm::PhysicalLength), (override));
    MOCK_METHOD(t::MovementResult, elevate, (dm::PhysicalLength), (override));
};

class MockMap : public dm::IMap3D {
public:
    MOCK_METHOD(t::VoxelOccupancy, atVoxel, (const dm::Position3D&), (const, override));
    MOCK_METHOD(t::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const dm::Position3D&), (const, override));
};

class MockMutableMap : public dm::IMutableMap3D {
public:
    MOCK_METHOD(t::VoxelOccupancy, atVoxel, (const dm::Position3D&), (const, override));
    MOCK_METHOD(t::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const dm::Position3D&), (const, override));
    MOCK_METHOD(void, set, (const dm::Position3D&, t::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
};

// IMappingAlgorithm has a data-carrying constructor; forward to it.
class MockMappingAlgorithm : public dm::IMappingAlgorithm {
public:
    MockMappingAlgorithm(const t::MissionConfigData& mission,
                         const t::LidarConfigData& lidar,
                         const t::DroneConfigData& drone,
                         const dm::IMap3D& output_map)
        : dm::IMappingAlgorithm(mission, lidar, drone, output_map) {}
    MOCK_METHOD(t::MappingStepCommand, nextStep,
                (const t::DroneState&, const t::LidarScanResult*), (override));
};

class MockDroneControl : public dm::IDroneControl {
public:
    MOCK_METHOD(t::DroneStepResult, step, (), (override));
    MOCK_METHOD(t::DroneState, state, (), (const, override));
};

class MockMissionControl : public dm::IMissionControl {
public:
    MOCK_METHOD(t::MissionRunResult, runMission, (), (override));
};

class MockSimulationRun : public dm::ISimulationRun {
public:
    MOCK_METHOD(t::SimulationResult, run, (), (override));
};

class MockSimulationRunFactory : public dm::ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<dm::ISimulationRun>, create,
                (const t::SimulationConfigData&, const t::MissionConfigData&,
                 const t::DroneConfigData&, const t::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

} // namespace dmtest
