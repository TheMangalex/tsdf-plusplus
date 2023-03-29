// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/core/object_volume.h"


using namespace voxblox;

ObjectVolume::ObjectVolume(float voxel_size, size_t voxels_per_side,
                           const Point centroid,
                           const SemanticClass& semantic_class)
    : tsdf_layer_(new Layer<TsdfVoxel>(voxel_size, voxels_per_side)),
      semantic_class_(semantic_class) {
  pose_ = Transformation(Rotation(), centroid);
  pos_ = pose_.getPosition();
}

void ObjectVolume::accumulateTransform(Transformation transform) {
  // T_G_O_t2 = (T_O_t1_O_t2) * T_G_O_t1
  pose_ = transform * pose_;
  std::cerr << "accumulate call" << std::endl;
  pose_ = Transformation(pose_.getRotation().normalize(), pose_.getPosition());
}

Block<TsdfVoxel>::Ptr ObjectVolume::allocateStorageAndGetBlockPtr(
    const BlockIndex& block_idx) {
  std::lock_guard<std::mutex> lock(temp_block_mutex_);

  typename Layer<TsdfVoxel>::BlockHashMap::iterator it =
      temp_block_map_.find(block_idx);
  if (it != temp_block_map_.end()) {
    return it->second;
  } else {
    auto insert_status = temp_block_map_.emplace(
        block_idx,
        std::make_shared<Block<TsdfVoxel>>(
            tsdf_layer_->voxels_per_side(), tsdf_layer_->voxel_size(),
            getOriginPointFromGridIndex(block_idx, tsdf_layer_->block_size())));

    CHECK(insert_status.second)
        << "Block already exists when allocating at " << block_idx.transpose();

    return insert_status.first->second;
  }
}

// NOT thread safe.
void ObjectVolume::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  Block<TsdfVoxel>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>&
           temp_block_pair : temp_block_map_) {
    tsdf_layer_->insertBlock(temp_block_pair);
  }

  Point p(0, 0, 0);
  float count = 0;

  // TODO this is not fast, rethink if this is good or to prone to errors?
  BlockIndexList blocks;
  tsdf_layer_->getAllAllocatedBlocks(&blocks);
  for (auto& block_idx : blocks) {
    auto block = tsdf_layer_->getBlockPtrByIndex(block_idx);
    for (uint i = 0; i < block->voxels_per_side() * block->voxels_per_side() * block->voxels_per_side(); i++) {
      if (block->isValidLinearIndex(i)) {
        auto weight = block->getVoxelByLinearIndex(i).weight;
        if (weight > 1.1) {
          auto point = block->computeCoordinatesFromLinearIndex(i);
          p += point * std::max(weight, 2.0f);
          count += std::max(weight, 2.0f);
        }
      }
    }
  }
  pos_ = p / count;
  //std::cerr << pos_ << std::endl;

  temp_block_map_.clear();
}
