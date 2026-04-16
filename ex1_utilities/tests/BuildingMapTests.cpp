#include "TestFramework.hpp"
#include "drone_mapper/BuildingMap.hpp"

DM_TEST(SparseBuildingMapStartsUnmappedInsideMissionAndOutOfBoundsOutside) {
  const dm::MissionConfig mission{-1, -1, 1, 1, 0, 1, 0, 0};
  dm::SparseBuildingMap map{mission};

  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{0, 0, 0})), static_cast<int>(dm::CellState::Unmapped));
  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{-1, -1, 1})), static_cast<int>(dm::CellState::Unmapped));
  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{2, 0, 0})), static_cast<int>(dm::CellState::OutOfBounds));
  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{0, 0, -1})), static_cast<int>(dm::CellState::OutOfBounds));
}

DM_TEST(SparseBuildingMapStoresValuesWithoutCorruptingNeighbors) {
  dm::SparseBuildingMap map{dm::MissionConfig{0, 0, 2, 2, 0, 0, 0, 0}};

  map.set(dm::Position{1, 1, 0}, dm::CellState::Occupied);
  map.set(dm::Position{2, 2, 0}, dm::CellState::Empty);

  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{1, 1, 0})), static_cast<int>(dm::CellState::Occupied));
  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{2, 2, 0})), static_cast<int>(dm::CellState::Empty));
  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{1, 2, 0})), static_cast<int>(dm::CellState::Unmapped));
}

DM_TEST(SparseBuildingMapIgnoresOutOfBoundsWrites) {
  dm::SparseBuildingMap map{dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}};

  map.set(dm::Position{5, 5, 5}, dm::CellState::Occupied);

  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{0, 0, 0})), static_cast<int>(dm::CellState::Unmapped));
}

DM_TEST(SparseBuildingMapAllowsOverwritingMappedState) {
  dm::SparseBuildingMap map{dm::MissionConfig{0, 0, 1, 0, 0, 0, 0, 0}};

  map.set(dm::Position{0, 0, 0}, dm::CellState::Empty);
  map.set(dm::Position{0, 0, 0}, dm::CellState::Occupied);

  DM_ASSERT_EQ(static_cast<int>(map.get(dm::Position{0, 0, 0})), static_cast<int>(dm::CellState::Occupied));
}

DM_TEST(SparseBuildingMapSerializeDenseUsesMissionRelativeOrdering) {
  dm::SparseBuildingMap map{dm::MissionConfig{5, 7, 6, 8, 1, 1, 0, 0}};

  map.set(dm::Position{5, 7, 1}, dm::CellState::Empty);
  map.set(dm::Position{6, 7, 1}, dm::CellState::Occupied);
  map.set(dm::Position{5, 8, 1}, dm::CellState::Occupied);

  const auto dense = map.serialize_dense();

  DM_ASSERT_EQ(dense.size(), 4u);
  DM_ASSERT_EQ(static_cast<int>(dense[0]), static_cast<int>(dm::CellState::Empty));
  DM_ASSERT_EQ(static_cast<int>(dense[1]), static_cast<int>(dm::CellState::Occupied));
  DM_ASSERT_EQ(static_cast<int>(dense[2]), static_cast<int>(dm::CellState::Occupied));
  DM_ASSERT_EQ(static_cast<int>(dense[3]), static_cast<int>(dm::CellState::Unmapped));
}
