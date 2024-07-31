#include "BVH.h"

#include <assert.h>
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

	std::println("Joints:");
	constexpr auto handle_joint = [](const BVH::Joint &joint) {
		const auto name                 = joint.name;
		const auto offset               = joint.GetOffset();
		const auto e_site               = joint.GetSite();
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
		const auto channel_string = channel_short_string();
		if (e_site) {
			const auto site = *e_site;
			std::println("{}\t{}\toffset({},{},{})\tsite({},{},{})\t{}", idx, name, offset[0], offset[1], offset[2], site[0], site[1], site[2], channel_string);
		} else {
			std::println("{}\t{}\toffset({},{},{})\t\t{}", idx, name, offset[0], offset[1], offset[2], channel_string);
		}
	};
	for (int i = 0; i < bvh.GetNumJoint(); i++) {
		const auto &joint = *bvh.GetJoint(i);
		handle_joint(joint);
	}
}
