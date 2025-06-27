#include <PoseLib/misc/colmap_models.h>
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    // Test the equirectangular camera model
    poselib::Camera camera("EQUIRECTANGULAR", {1920.0, 960.0}, 1920, 960);
    
    std::cout << "Testing Equirectangular Camera Model" << std::endl;
    std::cout << "Model ID: " << camera.model_id << std::endl;
    std::cout << "Model Name: " << camera.model_name() << std::endl;
    std::cout << "Width: " << camera.width << ", Height: " << camera.height << std::endl;
    
    // Test some projection/unprojection
    Eigen::Vector2d normalized_coord(0.5, 0.3);  // Some normalized coordinate
    Eigen::Vector2d pixel_coord;
    Eigen::Matrix2d jacobian;
    
    // Test projection
    camera.project(normalized_coord, &pixel_coord);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Normalized coord: (" << normalized_coord.transpose() << ")" << std::endl;
    std::cout << "Projected pixel: (" << pixel_coord.transpose() << ")" << std::endl;
    
    // Test projection with Jacobian  
    camera.project_with_jac(normalized_coord, &pixel_coord, &jacobian);
    std::cout << "Jacobian:\n" << jacobian << std::endl;
    
    // Test unprojection (should recover original normalized coordinates)
    Eigen::Vector2d recovered_coord;
    camera.unproject(pixel_coord, &recovered_coord);
    std::cout << "Recovered coord: (" << recovered_coord.transpose() << ")" << std::endl;
    
    // Check if we recovered the original coordinates (within tolerance)
    double error = (normalized_coord - recovered_coord).norm();
    std::cout << "Round-trip error: " << error << std::endl;
    
    if (error < 1e-10) {
        std::cout << "✓ Test PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "✗ Test FAILED" << std::endl;
        return 1;
    }
}