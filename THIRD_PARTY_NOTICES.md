# Third-party notices

The `sdk_api` directory is the supplied Odin SDK 3.3.0 (20260822), retained without source changes. The standalone https_client_test sample and generated compile_commands.json are excluded from this production commit; SDK implementation sources, headers and other examples remain unchanged. No top-level SDK license was supplied; this project does not grant or replace the vendor's rights. Redistributors must have the necessary vendor permission.

Bundled SDK dependencies retain their original notices: Mongoose is GPL-2.0-only or commercially licensed (see `sdk_api/3rdparty/mongoose/LICENSE`); FastCRC carries its upstream license; tinyxml2 and cJSON carry notices in their sources. Eigen, OpenSSL and FFmpeg are installed from Ubuntu packages and retain their distribution notices. This driver uses the FFmpeg libraries supplied by Ubuntu, not a separately configured proprietary codec binary.

Runtime images include the archive of the included unmodified SDK sources, its build scripts and the GPL-2 text in `/opt/odin2/share/odin2_ros_driver/vendor`. Driver corresponding source and build recipe are in the repository linked by the OCI source/revision labels on CI images. Keep these notices with redistributed artifacts. The root LICENSE applies only to newly authored driver code, not to the vendor SDK or dependencies.
