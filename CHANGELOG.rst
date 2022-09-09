^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package iris_lama_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2022-09-09)
------------------
* Trigger graph optimization when map is requested
* Add graph slam to install list

1.3.1 (2022-09-01)
------------------
* Add missing dependency

1.3.0 (2022-09-01)
------------------
* Import iris_lama as a plain cmake package
* Add a new node for graph slam
* Several improvements to the localization node
* Add support for slam2d transient mapping

1.2.0 (2021-04-10)
------------------
* Remove backlash from frame ids
* Fix laser orientation checks in the localization node
* Add launch file for localization node
* Remove backlash from global namespace topics
* Publish localization covariance

1.1.0 (2020-12-05)
------------------
* Add "truncate" parameters
* Add a service to trigger global localization
* Add a service to trigger localization non-motion updates
* Localization node can now subscribe to maps instead of just using the static_map service
* Enable TCP_NODELAY for laser scan subscribers to reduce communications delay
* Filter "map" transformations when mapping offline
* Fix inverted lidar
* Use C++14
* Fix eigen aligment issues

1.0.0 (2020-05-05)
------------------
* First official release.
