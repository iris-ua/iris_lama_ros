^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package iris_lama_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
