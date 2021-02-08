/**
 * @author Riley Bowyer
 * @date 24-08-2020
 *
 * @brief Projection Parameter library
 *  This library introduces:
 *    + Beam and Angle information class for projection
 *
 * @namespace uqr
 */

#ifndef PROJECTION_PARAMS_H
#define PROJECTION_PARAMS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace uqr {
    class ProjectionParams{
        public:
            /// Empty Constructor
            ProjectionParams();

            /// Copy Constructor
            ProjectionParams(const ProjectionParams& ortherPrams) = default;

            /// Copy Operator
            ProjectionParams& operator=(const ProjectionParams&) = default;

            /// Move Constructor
            ProjectionParams(ProjectionParams&&) = default;

            /// Move Operator
            ProjectionParams& operator=(ProjectionParams&&) = default;

            /// Constructor
            ProjectionParams(float min_angle, float max_angle, float step, bool beamCount = false);

            /**
            * Populate Angle Vector.
            *
            * Function to fill the internal angle vector. 
            */
            void fill_angles();
            
            /**
            * Get Closest Angle Index.
            *
            * Return the index of the angle closest to the provided angle.
            *
            * @param angle Angle to find index for.
            * @return Corresponding index.
            */
            int find_closest(float angle);

            /**
            * Index to Angle
            *
            * Return the corresponding angle.
            *
            * @param index Index to find angle for.
            * @return Correspdoning angle.
            */
            float from_index(int index);

            /**
            * Angle to Index
            *
            * Return the corresponding index.
            *
            * @param angle Angle to find index for.
            * @return Correspdoning index.
            */
            int from_angle(float angle);

            /**
            * Cosines of angle vector
            *
            * Return the cos of the angles stored in the angle vector
            *
            * @return Correspdoning cos vector.
            */
            std::vector<float> cosines_vector();

            /**
            * Sines of angle vector
            *
            * Return the sin of the angles stored in the angle vector
            *
            * @return Correspdoning sin vector.
            */
            std::vector<float> sines_vector();

            /**
            * Angle Vector Length
            *
            * Return the length of the internal angle vector.
            *
            * @return Correspdoning length.
            */
            int len();

            /**
            * Angle Vis.
            *
            * Print the angle vector to the screen.
            */
            void show_angles();

        private:
            /// Smallest Angle (Generally -FOV/2)
            float min_angle;

            /// Largest Angle (Generally FOV/2)
            float max_angle;

            /// Resolution - Angle Increment
            float step;

            /// Resolution - Number of Angles
            int num_beams;

            /// Angle Vector
            std::vector<float> angles;
            std::vector<float> sines;
            std::vector<float> cosines;
            
    };
} // NAMESPACE uqr
#endif //PROJECTION_PARAMS_H
