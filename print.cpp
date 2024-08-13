#include "BVH.h"

#include <cassert>
#include <iostream>
#include <format>
#include <print>
#include <sstream>

int main() {
	constexpr auto FILE_NAME = "/Users/crosstyan/Code/bvh_player/data/172_jump_4.bvh";
	auto bvh                 = BVH(FILE_NAME);
	if (not bvh.IsLoadSuccess()) {
		std::println("Failed to load BVH file {}", FILE_NAME);
		return 1;
	}
	std::println("loaded BVH file {}", FILE_NAME);
	std::println("Interval={}s", bvh.GetInterval());
	std::println("FPS={}", 1.0 / bvh.GetInterval());
	std::println("NumFrame={}", bvh.GetNumFrame());
	std::println("AnimationTime={}s", bvh.GetNumFrame() * bvh.GetInterval());
	assert(bvh.GetNumChannel() == bvh.GetStride());
	std::println("NumFrame={} NumChannel={} Stride={}", bvh.GetNumFrame(), bvh.GetNumChannel(), bvh.GetStride());

	constexpr auto sel_frame = 120;
	std::println("Info at Frame {}", sel_frame);
	std::println("Joints:");
	constexpr auto handle_joint = [](const BVH::Joint &joint) {
		const auto name                 = joint.name;
		const auto offset               = joint.GetOffset();
		const auto idx                  = joint.index;
		const auto &channels            = joint.GetChannels();
		const auto channel_short_string = [channels] {
			auto ss = std::stringstream{};
			for (const auto *const channel : channels) {
				auto s = std::format("{}({})", BVH::ShortStringify(channel->type), channel->index);
				ss << s;
			}
			return ss.str();
		};
		std::print("{}\t{}", idx, name);
		if (const auto e_parent = joint.GetParent()) {
			const auto &parent = *e_parent;
			std::print("\tP({})", parent.name);
		}
		std::print("\toffset({:.4},{:.4},{:.4})", offset[0], offset[1], offset[2]);
		if (const auto e_site = joint.GetEndSite()) {
			const auto site = *e_site;
			std::print("\tend({:.4},{:.4},{:.4})", site[0], site[1], site[2]);
		} else {
			std::print("\t");
		}
		const auto channel_string = channel_short_string();
		std::print("\t{}", channel_string);
		if (const auto e_rotation = joint.GetRotation(sel_frame)) {
			const auto rotation = *e_rotation;
			std::print("\trot({:.4},{:.4},{:.4})", rotation[0], rotation[1], rotation[2]);
		}
		if (const auto e_position = joint.GetPosition(sel_frame)) {
			const auto position = *e_position;
			std::print("\tpos({:.4},{:.4},{:.4})", position[0], position[1], position[2]);
		}
		std::print("\n");
	};
	for (const auto *j : bvh.GetJoints()) {
		handle_joint(*j);
	}
}
