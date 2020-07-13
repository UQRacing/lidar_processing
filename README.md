# Perception-ConesLiDAR
LiDAR based object detection, specifically focused on identifying cones.


# Contributing
## Gitflow
This repo has been setup to use Gitflow Workflow (more info [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)).

Instead of a single `master` branch, there are two main branches. `master` stores the release version, and `develop` serves as an integration branch for features.

Each new feature should have it's own branch `feature/name_of_feature`, and should branch off `develop`. When a feature is complete, please merge it into `develop`.

Pull requests must have been reviewed by at least one person before being completed. However, because this organisation is a free(?) account, this rule cannot be enforced.

## Dependencies
### pcl_ros
Information [here](http://wiki.ros.org/pcl_ros).
