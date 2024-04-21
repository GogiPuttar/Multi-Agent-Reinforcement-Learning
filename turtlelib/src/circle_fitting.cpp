#include <iostream>
#include <armadillo>
#include "turtlelib/circle_fitting.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{

    Circle circle_fitting(std::vector<turtlelib::Point2D> cluster)
    {
        // 1. Find the mean of the x and y coordinates: 
        double centroid_x = 0.0;
        double centroid_y = 0.0;
        double cluster_size = static_cast<double>(cluster.size());

        for (size_t i = 0; i < cluster.size(); i++)
        {
            centroid_x += cluster.at(i).x;
            centroid_y += cluster.at(i).y;
        }

        centroid_x /= cluster_size; // eq...(1)
        centroid_y /= cluster_size; // eq...(2)

        double xc_i = 0.0;
        double yc_i = 0.0;
        double zc_i = 0.0;
        double z_mean = 0.0;
        arma::mat Z{};

        for (size_t i = 0; i < cluster.size(); i++)
        {
            // 2. Shift the coordinates so that the centroid is at the origin:
            
            xc_i = cluster.at(i).x - centroid_x;          // eq...(3)
            yc_i = cluster.at(i).y - centroid_y;          // eq...(4)

            // 3. Compute z_i

            zc_i = xc_i*xc_i + yc_i*yc_i;                         

            // 4. Compute mean of z
            z_mean += zc_i;

            // 5. Form the data matrix from the n data points
            Z.insert_rows(i,arma::rowvec{zc_i, xc_i, yc_i, 1}); // eq...(6)
        }

        z_mean /= cluster_size;

        // 6. Form the Moment Matrix
        arma::mat M = (1/cluster_size)*Z.t()*Z;

        // 7,8. Form the constraint matrix for the "Hyperaccurate algebraic fit"
        arma::mat H(4, 4, arma::fill::zeros);
        H(0,0) = 8.0*z_mean;    // eq...(7)
        H(3,0) = 2.0;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;

        // 9. Compute the Singular Value Decomposition of Z = U Σ V^T
        arma::mat U{};
        arma::vec sigma{};
        arma::mat V{};
        arma::svd(U, sigma, V, Z);    // eq...(9)

        // 10. If the smallest singular value σ4 is less than 1e−12, then Let A be the the 4th column of the V matrix
        arma::colvec A{};
        if (sigma(3) < 10e-12)
        {
            A = V.col(3);
        }
        // 11. If σ4 > 1e−12  then let Y = V Σ V^T
        else
        {
            arma::mat Y = V*arma::diagmat(sigma)*trans(V);    // eq...(10)

            // Then find the eigenvalues and vectors of Q = Y H^−1 Y
            arma::mat Q = Y*H.i()*Y;

            arma::cx_vec eigval{};
            arma::cx_mat eigvec{};
            arma::eig_gen(eigval, eigvec, Q);

            arma::mat real_eigval = arma::real(eigval); // Select real eigenvalues.
            arma::mat real_eigvec = arma::real(eigvec); // and eigenvectors

            // Let A∗ be the eigenvector corresponding to the smallest positive eigenvalue of Q
            double min = 1e6;
            size_t min_index = 0;
            // if(arma::any(real_eigval > min)) 
            // {
            //     throw std::runtime_error("Eigenvalues are too large!");
            // }

            for (size_t i = 0; i < real_eigval.size(); i++)
            {
                if (real_eigval(i) < min && real_eigval(i) > 0.0) // Find the smallest positive eigval
                {
                    min = real_eigval(i);
                    min_index = i;
                }
            }
            arma::vec A_star = real_eigvec.col(min_index); // Extract eigvec that corresponds with the eigen value

            // Solve Y A = A∗ for A
            A = Y.i()*A_star;
        }

        // 12. Once we have A then the equation for the circle is
        double a = -A(1)/(2.0*A(0));    // eq...(12)
        double b = -A(2)/(2.0*A(0));    // eq...(13)
        double R = std::sqrt((A(1)*A(1) + A(2)*A(2) - 4.0*A(0)*A(3)) / (4.0*A(0)*A(0))); // eq...(14)

        // 13. We shifted the coordinate system, so the actual center is at
        double cx = a + centroid_x;
        double cy = b + centroid_y;

    return {cx, cy, R};
    }
}