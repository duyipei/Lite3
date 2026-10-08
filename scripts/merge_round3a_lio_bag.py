#!/usr/bin/env python3
"""Create a derived LiDAR+IMU rosbag2 SQLite database for Round 3A playback.

The acquisition database(s) are left untouched. Recorder timestamps are
preserved and globally sorted so Foxy can replay both sensor streams from one
database without introducing a second-player start offset.
"""

import argparse
import glob
import os
import re
import sqlite3
import sys


LIDAR_TOPIC = "/fast_livo2/livox/lidar"
IMU_TOPIC = "/fast_livo2/livox/imu"


def source_topic(connection, name):
    row = connection.execute(
        "SELECT name, type, serialization_format, offered_qos_profiles "
        "FROM topics WHERE name = ?", (name,)
    ).fetchone()
    if row is None:
        raise RuntimeError("required topic is absent: {}".format(name))
    return row


def natural_key(path):
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", path)]


def existing_files(patterns):
    paths = []
    for pattern in patterns:
        paths.extend(glob.glob(pattern))
    return sorted(set(os.path.abspath(path) for path in paths), key=natural_key)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_dir",
        help="Round 3A run directory containing sensors/ or legacy lidar/ and camera_imu/",
    )
    parser.add_argument("output_db", help="new derived .db3 path; must not already exist")
    args = parser.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    unified_dbs = existing_files(
        [
            os.path.join(run_dir, "sensors", "sensors_*.db3"),
            os.path.join(
                run_dir,
                "sensors",
                os.path.basename(run_dir) + "_sensors_*.db3",
            ),
        ]
    )
    if unified_dbs:
        lidar_dbs = unified_dbs
        camera_dbs = unified_dbs
    else:
        lidar_dbs = existing_files([os.path.join(run_dir, "lidar", "lidar_*.db3")])
        camera_dbs = existing_files(
            [
                os.path.join(run_dir, "camera_imu", "camera_imu_*.db3"),
                # Legacy tmpfs recorder layout retained for existing field bags.
                os.path.join(
                    run_dir,
                    "camera_imu",
                    os.path.basename(run_dir) + "_camera_imu_*.db3",
                ),
            ]
        )
    output_db = os.path.abspath(args.output_db)

    if not lidar_dbs:
        raise RuntimeError(
            "lidar database does not exist under sensors/ or lidar/"
        )
    if not camera_dbs:
        raise RuntimeError(
            "camera/IMU database does not exist under sensors/ or camera_imu/"
        )
    for path in lidar_dbs + camera_dbs:
        if not os.path.isfile(path):
            raise RuntimeError("source database does not exist: {}".format(path))
    if os.path.exists(output_db):
        raise RuntimeError("refusing to overwrite existing output: {}".format(output_db))
    os.makedirs(os.path.dirname(output_db), exist_ok=True)

    lidar = sqlite3.connect("file:{}?mode=ro&immutable=1".format(lidar_dbs[0]), uri=True)
    camera = sqlite3.connect("file:{}?mode=ro&immutable=1".format(camera_dbs[0]), uri=True)
    lidar_topic = source_topic(lidar, LIDAR_TOPIC)
    imu_topic = source_topic(camera, IMU_TOPIC)
    lidar.close()
    camera.close()

    destination = sqlite3.connect(output_db)
    try:
        destination.execute("PRAGMA journal_mode=DELETE")
        destination.execute("PRAGMA synchronous=NORMAL")
        destination.execute(
            "CREATE TABLE topics("
            "id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
            "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
        )
        destination.execute(
            "CREATE TABLE messages("
            "id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, "
            "timestamp INTEGER NOT NULL, data BLOB NOT NULL)"
        )
        destination.execute("INSERT INTO topics VALUES(1, ?, ?, ?, ?)", lidar_topic)
        destination.execute("INSERT INTO topics VALUES(2, ?, ?, ?, ?)", imu_topic)
        if lidar_dbs == camera_dbs:
            for source_db in lidar_dbs:
                destination.execute("ATTACH DATABASE ? AS sensor_source", (source_db,))
                destination.execute(
                    "INSERT INTO messages(topic_id, timestamp, data) "
                    "SELECT topic_id, timestamp, data FROM ("
                    "SELECT 1 AS topic_id, timestamp, data FROM sensor_source.messages "
                    "WHERE topic_id=(SELECT id FROM sensor_source.topics WHERE name=?) "
                    "UNION ALL "
                    "SELECT 2 AS topic_id, timestamp, data FROM sensor_source.messages "
                    "WHERE topic_id=(SELECT id FROM sensor_source.topics WHERE name=?)"
                    ") ORDER BY timestamp",
                    (LIDAR_TOPIC, IMU_TOPIC),
                )
                destination.commit()
                destination.execute("DETACH DATABASE sensor_source")
        else:
            for source_db in lidar_dbs:
                destination.execute("ATTACH DATABASE ? AS lidar_source", (source_db,))
                destination.execute(
                    "INSERT INTO messages(topic_id, timestamp, data) "
                    "SELECT 1, timestamp, data FROM lidar_source.messages "
                    "WHERE topic_id=(SELECT id FROM lidar_source.topics WHERE name=?) "
                    "ORDER BY timestamp",
                    (LIDAR_TOPIC,),
                )
                destination.commit()
                destination.execute("DETACH DATABASE lidar_source")
            for source_db in camera_dbs:
                destination.execute("ATTACH DATABASE ? AS camera_source", (source_db,))
                destination.execute(
                    "INSERT INTO messages(topic_id, timestamp, data) "
                    "SELECT 2, timestamp, data FROM camera_source.messages "
                    "WHERE topic_id=(SELECT id FROM camera_source.topics WHERE name=?) "
                    "ORDER BY timestamp",
                    (IMU_TOPIC,),
                )
                destination.commit()
                destination.execute("DETACH DATABASE camera_source")
        destination.execute("CREATE INDEX timestamp_idx ON messages(timestamp)")
        destination.execute("CREATE INDEX topic_timestamp_idx ON messages(topic_id, timestamp)")
        destination.commit()

        rows = destination.execute(
            "SELECT topics.name, count(*), min(timestamp), max(timestamp) "
            "FROM messages JOIN topics ON messages.topic_id=topics.id "
            "GROUP BY topics.id ORDER BY topics.id"
        ).fetchall()
        integrity = destination.execute("PRAGMA integrity_check").fetchone()[0]
    except Exception:
        destination.close()
        try:
            os.unlink(output_db)
        except OSError:
            pass
        raise
    destination.close()

    print("output={}".format(output_db))
    print("integrity={}".format(integrity))
    for name, count, first, last in rows:
        duration = (last - first) / 1e9 if count > 1 else 0.0
        rate = (count - 1) / duration if duration > 0 else 0.0
        print("{} count={} span={:.3f}s rate={:.3f}Hz".format(name, count, duration, rate))
    if integrity != "ok":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
