#include <PoseLib/misc/colmap_models.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

bool test_round_trip(const poselib::Camera& camera, const Eigen::Vector2d& normalized_coord, double tolerance = 1e-10) {
    Eigen::Vector2d pixel_coord, recovered_coord;
    
    // Project to pixels
    camera.project(normalized_coord, &pixel_coord);
    
    // Unproject back to normalized coordinates
    camera.unproject(pixel_coord, &recovered_coord);
    
    // Check round-trip error
    double error = (normalized_coord - recovered_coord).norm();
    return error < tolerance;
}

bool test_jacobian_numerical(const poselib::Camera& camera, const Eigen::Vector2d& x, double delta = 1e-8) {
    Eigen::Vector2d xp, xp_dx, xp_dy;
    Eigen::Matrix2d jac_analytical, jac_numerical;
    
    // Get analytical Jacobian
    camera.project_with_jac(x, &xp, &jac_analytical);
    
    // Compute numerical Jacobian
    Eigen::Vector2d x_dx = x + Eigen::Vector2d(delta, 0);
    Eigen::Vector2d x_dy = x + Eigen::Vector2d(0, delta);
    
    camera.project(x_dx, &xp_dx);
    camera.project(x_dy, &xp_dy);
    
    jac_numerical.col(0) = (xp_dx - xp) / delta;
    jac_numerical.col(1) = (xp_dy - xp) / delta;
    
    // Check difference
    double error = (jac_analytical - jac_numerical).norm();
    return error < 1e-4; // More relaxed tolerance for numerical differentiation
}

int main() {
    poselib::Camera camera("EQUIRECTANGULAR", {1920.0, 960.0}, 1920, 960);
    
    std::cout << "Comprehensive Equirectangular Camera Model Test" << std::endl;
    std::cout << "===============================================" << std::endl;
    
    // Test 1: Basic round-trip tests
    std::vector<Eigen::Vector2d> test_coords = {
        {0.0, 0.0},    // Center
        {0.5, 0.3},    // Arbitrary point
        {-0.2, -0.8},  // Negative coordinates
        {1.0, 0.5},    // Edge case
        {0.1, 0.1}     // Small values
    };
    
    std::cout << "\nTest 1: Round-trip accuracy" << std::endl;
    bool all_round_trips_passed = true;
    for (size_t i = 0; i < test_coords.size(); i++) {
        bool passed = test_round_trip(camera, test_coords[i]);
        std::cout << "  Coord " << i+1 << " (" << test_coords[i].transpose() << "): " 
                  << (passed ? "✓" : "✗") << std::endl;
        all_round_trips_passed &= passed;
    }
    
    // Test 2: Jacobian accuracy  
    std::cout << "\nTest 2: Jacobian accuracy (analytical vs numerical)" << std::endl;
    bool all_jacobians_passed = true;
    for (size_t i = 0; i < test_coords.size(); i++) {
        bool passed = test_jacobian_numerical(camera, test_coords[i]);
        std::cout << "  Coord " << i+1 << " (" << test_coords[i].transpose() << "): " 
                  << (passed ? "✓" : "✗") << std::endl;
        all_jacobians_passed &= passed;
    }
    
    // Test 3: Edge cases - extreme coordinates
    std::cout << "\nTest 3: Edge cases" << std::endl;
    std::vector<Eigen::Vector2d> edge_coords = {
        {5.0, 2.0},    // Large coordinates
        {-3.0, -1.5},  // Large negative coordinates
        {0.001, 0.001} // Very small coordinates
    };
    
    bool all_edge_cases_passed = true;
    for (size_t i = 0; i < edge_coords.size(); i++) {
        bool passed = test_round_trip(camera, edge_coords[i], 1e-8); // Slightly relaxed tolerance for edge cases
        std::cout << "  Edge coord " << i+1 << " (" << edge_coords[i].transpose() << "): " 
                  << (passed ? "✓" : "✗") << std::endl;
        all_edge_cases_passed &= passed;
    }
    
    // Test 4: Specific mathematical properties
    std::cout << "\nTest 4: Mathematical properties" << std::endl;
    
    // Test that the center of the image maps correctly
    Eigen::Vector2d center_normalized(0, 0);
    Eigen::Vector2d center_pixel;
    camera.project(center_normalized, &center_pixel);
    
    // For (0,0) normalized coord, we expect it to map to the front of the sphere
    // which should be around the center of the equirectangular image
    bool center_test = std::abs(center_pixel(0) - 960.0) < 1.0; // Should be near width/2
    std::cout << "  Center mapping (" << center_pixel.transpose() << " ≈ [960, y]): " 
              << (center_test ? "✓" : "✗") << std::endl;
    
    // Test focal length method
    double focal = camera.focal();
    bool focal_test = std::abs(focal - 1920.0) < 1e-10;
    std::cout << "  Focal length (" << focal << " = 1920): " 
              << (focal_test ? "✓" : "✗") << std::endl;
    
    // Final result
    bool all_passed = all_round_trips_passed && all_jacobians_passed && all_edge_cases_passed && center_test && focal_test;
    
    std::cout << "\n===============================================" << std::endl;
    std::cout << "Overall result: " << (all_passed ? "✓ ALL TESTS PASSED" : "✗ SOME TESTS FAILED") << std::endl;
    
    return all_passed ? 0 : 1;
}