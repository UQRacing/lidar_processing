# ugly code to generate uglier code :)
# copyright (c) 2169 your mum. no rights reserved

if __name__ == "__main__":
    # min
    for coord in ["x", "y", "z"]:
        # handle.getParam("/lidar_cone_detector/dbscan_min_pts", dbscanMinPts);
        print(f"handle.getParam(\"/lidar_cone_detector/bbox_min_{coord}\", bboxMin{coord.upper()});")

    # max
    for coord in ["x", "y", "z"]:
        print(f"handle.getParam(\"/lidar_cone_detector/bbox_max_{coord}\", bboxMax{coord.upper()});")

    # ddynamic_reconfigure
    for thing in ["min", "max"]:
        for coord in ["x", "y", "z"]:
            # ddr.registerVariable("planeNumIters", &planeNumIters, "Plane num iterations", 10, 512);
            print(f"ddr.registerVariable(\"bbox_{thing}_{coord}\", &bbox{thing.title()}{coord.upper()}, \"LiDAR bounding box {thing} {coord}\", -100.0, 100.0);")