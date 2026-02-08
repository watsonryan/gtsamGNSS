/**
 *  @file   GNSSPostfit.cpp
 *  @author Watson
 *  @brief  Implementation file for GNSS postfit analysis
 **/

#include <gtsam/gnssNavigation/GnssPostfit.h>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <numeric>


namespace gtsam {

std::vector<double> getResiduals(const Point3& nomXYZ, const Values& results, const std::vector<rnxData>& data){
        /*
         * inputs ::
         *   nomXYZ --> nominal ECEF platform position. [Point3]
         *   results --> optimized solution [Values]
         *   data --> observed GNSS data [vector<rnxData>]
         * output ::
         *   postFit --> GNSS postfit residuals [vector<double>]
         */
        std::size_t epoch = 0;
        std::size_t epochKey = 0;
        std::vector<double> postFit;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        for (const auto& key_value : result_poses) {
                nonBiasStates q = key_value.value;
                while (epoch < data.size() && epochKey == static_cast<std::size_t>(data[epoch].epoch)) {
                        Vector h = obsMap(data[epoch].sat_xyz, nomXYZ, 1);
                        double est = h.transpose() * q;
                        double residual = est - (data[epoch].range_lc - data[epoch].computed_range);
                        postFit.push_back(residual);
                        epoch++;
                }
                epochKey++;
        }
        return postFit;
}

void writeResiduals(const std::vector<double>& postfitResiduals, const std::string& outputFile, const std::vector<std::string>& index) {
        /*
         * inputs ::
         *  postfitResiduals --> GNSS postfit residuals [vector<double>]
         *  outputFile --> specify the destination file name [string]
         *  index --> unique idetifier for GNSS residuals (i.e. SVN) [ vector<string>]
         * outputs ::
         *  std out
         */
        std::ofstream outFile(outputFile.c_str());
        const std::size_t count = std::min(postfitResiduals.size(), index.size());
        for (std::size_t i = 0; i < count; i++) {
                outFile << index[i] << " " << " " <<  postfitResiduals[i] << std::endl;
        }
}

std::vector<int> markResiduals(const std::vector<double>& postfitResdiuals, double threshold) {
        /*
         * inputs ::
         *  postfitResiduals --> GNSS postfit residuals [vector<double>]
         *  threshold --> theshold for labeling residual as outlier.
         * outputs ::
         *  markedOutliers --> label vector to specify if observable is an outlier [vector<int>]
         */
        std::vector<std::size_t> residuals(postfitResdiuals.size());
        std::vector<int> markedOutliers;
        std::iota(residuals.begin(), residuals.end(), 0);
        std::copy_if(residuals.begin(), residuals.end(), std::back_inserter(markedOutliers), [=](std::size_t i) { return std::abs(postfitResdiuals[i]) > threshold; });
        return markedOutliers;
}

}
