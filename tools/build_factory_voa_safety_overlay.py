#!/usr/bin/env python3
"""Copy factory VOA into an isolated workspace and preserve observation masks.

No chassis topic, service, launch or velocity-selection code is changed. The
factory installation is never overwritten. Run colcon separately on Jetson.
"""
import argparse
from pathlib import Path
import shutil


def patch_grid(source):
    if 'observation_validity' in source:
        raise ValueError('observation mask already present; review rather than patch twice')
    source = source.replace('#include <chrono>', '#include <chrono>\n#include <limits>')
    needle = 'grid_map_.add("elevation");'
    if source.count(needle) != 1:
        raise ValueError('unrecognized factory layer construction')
    source = source.replace(needle, needle+'\n        grid_map_.add("observation_validity");')
    needle = '    void Process(){'
    if source.count(needle) != 1:
        raise ValueError('unrecognized factory Process')
    source = source.replace(needle, needle+'''
        // CV conversion writes unobserved NaNs as the minimum image value.
        // Preserve independent observation support across dilation/conversion.
        const auto observed = grid_map_.get("elevation").array().isFinite().eval();
        grid_map_.get("observation_validity") = observed.cast<float>().matrix();
        if (!observed.any()) return;
''')
    needle = '\n    grid_map::GridMap &GetGridMap(){'
    if source.count(needle) != 1:
        raise ValueError('unrecognized factory Process boundary')
    # Insert before the final Process closing brace, after CPU/CUDA branches.
    at = source.index(needle)
    closing = source.rfind('    }', 0, at)
    source = source[:closing]+'''
        auto &elevation = grid_map_.get("elevation");
        for (int row = 0; row < elevation.rows(); ++row) {
            for (int col = 0; col < elevation.cols(); ++col) {
                if (!observed(row, col))
                    elevation(row, col) = std::numeric_limits<float>::quiet_NaN();
            }
        }
'''+source[closing:]
    old = 'rclcpp::Time time(stamp_ref_pc.sec, RCL_STEADY_TIME);'
    source = source.replace(old,'rclcpp::Time time(stamp_ref_pc.sec, stamp_ref_pc.nanosec, RCL_STEADY_TIME);')
    if 'rclcpp::Time time(stamp_ref_pc.sec, stamp_ref_pc.nanosec, RCL_STEADY_TIME);' not in source:
        raise ValueError('unrecognized timestamp construction')
    return source


def patch_latest_cloud(processor, mapper):
    changes = [
        ('point_cloud_in_topic_name, 10,', 'point_cloud_in_topic_name, rclcpp::QoS(rclcpp::KeepLast(1)),', 'processor'),
        ('point_cloud_out_topic_name, 10)', 'point_cloud_out_topic_name, 1)', 'processor'),
        ('            topic_name,\n                10,', '            topic_name,\n                rclcpp::QoS(rclcpp::KeepLast(1)),', 'mapper'),
        ('cloud_buffers_.at(i)->emplace_back(msg);',
         'cloud_buffers_.at(i)->clear(); // only the latest camera frame is relevant\n                            cloud_buffers_.at(i)->emplace_back(msg);', 'mapper')]
    for before,after,target in changes:
        text = processor if target == 'processor' else mapper
        if text.count(before) != 1:
            raise ValueError('unrecognized factory queue contract: '+before)
        text = text.replace(before,after)
        if target == 'processor': processor = text
        else: mapper = text
    return processor,mapper


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('factory_package', type=Path)
    p.add_argument('workspace', type=Path)
    a = p.parse_args()
    if a.workspace.exists():
        p.error('new workspace required; do not overwrite previous evidence')
    source = a.factory_package/'src/MyGridMap.hpp'
    patched = patch_grid(source.read_text())
    processor,mapper = patch_latest_cloud((a.factory_package/'src/PointCloudProcessor.hpp').read_text(),
                                          (a.factory_package/'src/GridMapper.hpp').read_text())
    target = a.workspace/'src/voa'
    shutil.copytree(a.factory_package,target)
    (target/'src/MyGridMap.hpp').write_text(patched)
    (target/'src/PointCloudProcessor.hpp').write_text(processor)
    (target/'src/GridMapper.hpp').write_text(mapper)
    print('Prepared isolated factory perception patch:',target)


if __name__=='__main__':
    main()
