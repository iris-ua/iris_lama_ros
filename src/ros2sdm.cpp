/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2021-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <libgen.h>

#include <iostream>
#include <cstdlib>

#include <yaml-cpp/yaml.h>

#include <lama/types.h>
#include <lama/print.h>
#include <lama/time.h>
#include <lama/image.h>
#include <lama/image_io.h>
#include <lama/sdm/simple_occupancy_map.h>
#include <lama/sdm/dynamic_distance_map.h>
#include <lama/sdm/export.h>


struct YAMLWrapper {

    struct InnerWrapper {

        InnerWrapper(const std::string& key, const YAML::Node& root)
            : key(key), root(root)
        {}

        InnerWrapper(const InnerWrapper& other)
            : key(other.key), root(other.root)
        {}

        template <typename T>
        T as(){
            try {
                T value = root[key].as<T>();
                return value;
            } catch (YAML::Exception& e) {
                std::cerr << "Error accessing '" << key << "'" << std::endl
                           << e.what() << std::endl;
                exit(EXIT_FAILURE);
            }
            return T();
        }

        std::string key;
        const YAML::Node& root;
    };

    YAMLWrapper(const std::string& filename)
    {
        try {
            root = YAML::LoadFile(filename);
        } catch (YAML::Exception & e){
            std::cerr << "Unable to open '" << filename << "': " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    InnerWrapper operator[](const std::string& key) const
    { return InnerWrapper(key, root); }

    YAML::Node root;
};

int main(int argc, const char** argv)
{
    if (argc != 2){
        std::cerr << "Convert an occupancy map to LaMa's sparse dense map binary format." << std::endl
                  << "It will generate 'occ.sdm' and 'dm.sdm', the maps necessary for localization." << std::endl
                  << std::endl
                  << "USAGE: ros2sdm <map.yaml>" << std::endl;
        return EXIT_FAILURE;
    }

    YAMLWrapper config(argv[1]);
    std::string image_name  = config["image"].as<std::string>();
    double resolution       = config["resolution"].as<double>();
    int negate              = config["negate"].as<int>();
    double free_thresh      = config["free_thresh"].as<double>();
    double occupied_thresh  = config["occupied_thresh"].as<double>();
    std::vector<double> origin = config["origin"].as<std::vector<double>>();

    char abs_path[PATH_MAX+1];
    char* ptr = realpath(argv[1], abs_path);
    if (ptr == nullptr){
        std::cerr << "Failed to resolve yaml path. Make sure the file exists" << std::endl;
        return EXIT_FAILURE;
    }

    std::string base_dir(dirname(ptr));

    //--
    lama::Timer timer;
    double elapsed;

    lama::print("Loading image...\n");

    timer.start();
    lama::Image image;
    bool ok = lama::image_read(image, base_dir + "/" + image_name);
    if (!ok){
        std::cerr << "Unable to read image." << std::endl;
        return EXIT_FAILURE;
    }
    elapsed = timer.elapsed().toSec();
    lama::print(" - load time: %.2fs\n - width: %d\n - height: %d\n - channel(s): %d\n",
                elapsed, image.width, image.height, image.channels);

    // Generate the maps
    lama::print("Generating maps...\n");


    timer.start();
    lama::SimpleOccupancyMap occupancy_map(resolution);
    lama::DynamicDistanceMap distance_map(resolution);

    uint32_t width = image.width;
    uint32_t height= image.height;

    for (unsigned int j = 0; j < height; ++j) {
        for (unsigned int i = 0; i < width;  ++i) {
            Eigen::Vector3d coords;
            coords.x() = origin[0] + i * resolution;
            coords.y() = origin[1] + j * resolution;

            unsigned char value = image.data[i + (height - j - 1)*width];
            if ( negate ) value = 255 - value;

            double occ = (255 - value) / 255.0;

            if ( occ < free_thresh ){
                occupancy_map.setFree(coords);
            } else if (occ > occupied_thresh) {
                occupancy_map.setOccupied(coords);
                distance_map.addObstacle(distance_map.w2m(coords));
            }
        }// end for
    }//end for

    distance_map.update();
    elapsed = timer.elapsed().toSec();
    lama::print(" - time: %.2fs\n", elapsed);

    std::cout << "Saving maps..." << std::endl;

    timer.start();
    lama::print(" - Occupancy -> occ.sdm\n");
    occupancy_map.write("occ.sdm");

    lama::print(" - Distance -> dm.sdm\n");
    distance_map.write("dm.sdm");

    elapsed = timer.elapsed().toSec();
    lama::print(" - time: %.2fs\n", elapsed);

    std::cout << "Finished." << std::endl;
    return EXIT_SUCCESS;
}
