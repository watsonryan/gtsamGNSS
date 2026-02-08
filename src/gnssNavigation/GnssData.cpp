/**
 * @file   GnssData.cpp
 * @brief  Tools required to read/write GNSS data
 *  @author Watson
 */

#include <iomanip>      // std::setprecision
#include <random>

#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssErrors.h>

namespace gtsam {
namespace {

void checkIndexSize(const std::size_t index, const std::size_t size, const char* name) {
        if (index >= size) {
                throw GnssIndexMismatchError(name, index, size);
        }
}

std::ifstream openInputOrThrow(const std::string& path) {
        std::ifstream is(path.c_str());
        if (!is.is_open()) {
                throw GnssDataFileOpenError(path);
        }
        return is;
}

std::ofstream openOutputOrThrow(const std::string& path) {
        std::ofstream os(path.c_str());
        if (!os.is_open()) {
                throw GnssOutputFileOpenError(path);
        }
        return os;
}

std::mt19937 makeRng(const std::optional<std::uint32_t>& seed) {
        if (seed.has_value()) {
                return std::mt19937(*seed);
        }
        std::random_device rd;
        std::seed_seq seed_seq{rd(), rd(), rd(), rd()};
        return std::mt19937(seed_seq);
}

void ensureFullyParsed(const std::ifstream& is, const std::string& path) {
        if (!is.eof() && is.fail()) {
                throw GnssParseError(path, "truncated or malformed record encountered");
        }
}

}  // namespace

std::vector<rnxData> readGNSS(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        std::vector<rnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        std::ifstream is = openInputOrThrow(fileLoc);

        std::string satType;
        int svn, count;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
        while (is >> week >> sow  >> count
               >> satType >> svn >> rangeLC >> phaseLC
               >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del) {
                data.push_back(rnxData{
                    sow, count, svn, Point3(satX, satY, satZ),
                    (rho - cb + rel + grav_delay + trop_slant - satPC),
                    (rangeLC - c1Del + c2Del), (phaseLC - windup * 0.017),
                    static_cast<int>(break_flag)});
                // 0.01702215881 == LC wavelength/2*pi
        }
        ensureFullyParsed(is, fileLoc);
        return data;
}


std::vector<rnxData> readGNSS_SingleFreq(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        std::vector<rnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        std::ifstream is = openInputOrThrow(fileLoc);

        std::string satType;
        int svn, count;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, range, phase, iono_slant;
        while (is >> week >> sow  >> count
               >> satType >> svn >> range >> phase
               >> rho >> cb >> rel >> grav_delay
               >> trop_slant >>  iono_slant >> windup >> satPC >> satX
               >> satY >> satZ >> break_flag >> c1Del) {
                data.push_back(rnxData{
                    sow, count, svn, Point3(satX, satY, satZ),
                    (rho - cb + rel + grav_delay + trop_slant - iono_slant - satPC),
                    (range - c1Del), (phase - windup * 0.017),
                    static_cast<int>(break_flag)});
                // 0.01702215881 == LC wavelength/2*pi
        }
        ensureFullyParsed(is, fileLoc);
        return data;
}


std::vector<rnxDataWEl> readGNSS_SingleFreqTmp(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        std::vector<rnxDataWEl> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        std::ifstream is = openInputOrThrow(fileLoc);

        std::string satType;
        int svn, count;
        double grav_delay, windup, satPC, trop_slant, c1Del, c2Del, el, break_flag;
        double week, sow, satX, satY, satZ, rho, cb, rel, range, phase, iono_slant;
        while (is >> week >> sow  >> count
               >> satType >> svn >> range >> phase
               >> rho >> cb >> rel >> grav_delay
               >> trop_slant >>  iono_slant >> windup >> satPC >> satX
               >> satY >> satZ >> el >> break_flag >> c1Del) {
                data.push_back(rnxDataWEl{
                    sow, count, svn, Point3(satX, satY, satZ),
                    (rho - cb + rel + grav_delay + trop_slant - iono_slant - satPC),
                    (range - c1Del), (phase - windup * 0.017), break_flag, el});
                // 0.01702215881 == LC wavelength/2*pi
        }
        ensureFullyParsed(is, fileLoc);
        return data;
}


std::vector<faultyRnxData> readGNSSFaulty(const std::string &fileLoc, const double &mean, const double &stdDev, const double &percentFaulty, const std::optional<std::uint32_t>& seed) {
        /*
           inputs ::
           fileLoc ---> path to data file
           mean --> mean of distribution to generate observation faults
           stdDev --> 1 sigma of observation fault distribution
           percentFaulty --> number of faults to add to data set. scale == [0,1]
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        std::vector<faultyRnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        std::ifstream is = openInputOrThrow(fileLoc);

        std::mt19937 rng = makeRng(seed);
        std::normal_distribution<double> dist1(mean, stdDev);
        std::uniform_real_distribution<double> coin(0.0, 1.0);

        std::string satType;
        int svn, count, faultInd=0;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
        while (is >> week >> sow  >> count
                  >> satType >> svn >> rangeLC >> phaseLC
                  >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del) {
                if (coin(rng) < percentFaulty) {
                        if (coin(rng) > 0.5) {
                                rho += dist1(rng);
                                // rangeLC += dist1(gen1);
                                // phaseLC += dist2(gen2);
                        }
                        else {
                                rho -= dist1(rng);
                                // rangeLC -= dist1(gen1);
                                // phaseLC -= dist2(gen2);
                        }
                        faultInd = 1;
                }
                data.push_back(faultyRnxData{
                    sow, count, svn, Point3(satX, satY, satZ),
                    (rho - cb + rel + grav_delay + trop_slant - satPC),
                    (rangeLC - c1Del + c2Del), (phaseLC - windup * 0.017),
                    static_cast<int>(break_flag), faultInd});
                faultInd = 0;
        }
        ensureFullyParsed(is, fileLoc);
        return data;
}

std::vector<faultyRnxData> readGNSSOracle(const std::string &fileLoc, const double &mean, const double &stdDev, const double &percentFaulty, const std::optional<std::uint32_t>& seed) {
        /*
           inputs ::
           fileLoc ---> path to data file
           mean --> mean of distribution to generate observation faults
           stdDev --> 1 sigma of observation fault distribution
           percentFaulty --> number of faults to add to data set. scale == [0,1]
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        std::vector<faultyRnxData> data;
        (void)mean;
        (void)stdDev;
        std::string data_file = findExampleDataFile(fileLoc);
        std::ifstream is = openInputOrThrow(data_file);

        std::mt19937 rng = makeRng(seed);
        std::uniform_real_distribution<double> coin(0.0, 1.0);

        int svn, count, faultInd=0;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
        while (is >> week >> sow  >> count
                  >> svn >> rangeLC >> phaseLC
                  >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del) {
                if (coin(rng) > percentFaulty) {
                        data.push_back(faultyRnxData{
                            sow, count, svn, Point3(satX, satY, satZ),
                            (rho - cb + rel + grav_delay + trop_slant - satPC),
                            (rangeLC - c1Del + c2Del), (phaseLC - windup * 0.017),
                            static_cast<int>(break_flag), faultInd});
                        faultInd = 0;
                }
                else { continue; }
        }
        ensureFullyParsed(is, data_file);
        return data;
}
void writeStates(const Values& results, const std::vector<std::string>& timeIndex, const std::string& outputFile){
        /*
           inputs ::
           results -->
           outputFile --> name of file to write state est. to. [string]
         */
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** GNSS State Vector\n\n *time (sec), dx, dy, dz, dCB, dTz \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        for (const auto& key_value : result_poses)
        {
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), timeIndex.size(), "timeIndex");
                nonBiasStates p = key_value.value;
                outFile << timeIndex[index]
                        << " "  << p.x() << " " << p.y()
                        << " "  << p.z() << " " << p.cb()
                        << " " << p.tz() << std::endl;
        }
}

void writeNavFrame(const Values& results, const Point3& nom, const std::vector<std::string>& timeIndex, const std::string& outputFile){
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** ENU\n\n *time (sec), e (m), n (m), u (m)\n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        for (const auto& key_value : result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                Point3 enu = xyz2enu(ecef,nom);
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), timeIndex.size(), "timeIndex");
                outFile << timeIndex[index] << " " << enu.x()
                        << " " << enu.y() << " " << enu.z() << std::endl;

        }
}

void writeEarthFrame(const Values& results, const Point3& nom, const std::vector<std::string>& timeIndex, const std::string& outputFile){
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** ECEF \n\n *time (sec), prn, X (m), Y (m), Z (m) \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        for (const auto& key_value : result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), timeIndex.size(), "timeIndex");
                outFile << timeIndex[index] << " " << std::setprecision(10) << ecef.x()
                        << " " << ecef.y() << " " << ecef.z() << std::endl;

        }
}

void writeSwitches(const Values& results, const std::string& outputFile, const std::vector<std::string>& switchIndex){
        /*
           inputs ::
           results --> optimizer output
           outputFile --> name of file to write switch states to [string]
           Optional ::
           switchIndex --> index by epoch and visible satellite (i.e. obs 4 would be Switch_0_4) [vector]
         */
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** Switch Values\n\n *time (sec), prn, switch \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<vertigo::SwitchVariableLinear> result_switches = results.filter<vertigo::SwitchVariableLinear>();
        for (const auto& key_value : result_switches) {
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), switchIndex.size(), "switchIndex");
                outFile << switchIndex[index] << " " <<  key_value.value.value() << std::endl;
        }
}


void writeSwitchPair(const Values& results, const std::string& outputFile, const std::vector<std::string>& switchIndex){
        /*
           inputs ::
           results --> optimizer output
           outputFile --> name of file to write switch states to [string]
           Optional ::
           switchIndex --> index by epoch and visible satellite (i.e. obs 4 would be Switch_0_4) [vector]
         */
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** Switch Values \n\n *time (sec), prn, range switch, phase switch \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<vertigo::SwitchPairLinear> result_switches = results.filter<vertigo::SwitchPairLinear>();
        for (const auto& key_value : result_switches) {
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), switchIndex.size(), "switchIndex");
                outFile << switchIndex[index] << " " << key_value.value.a() << " " << key_value.value.b() << std::endl;
        }
}


void writeAmbiguity(const Values& results, const std::string& outputFile, const std::vector<std::string>& satIndex){
        std::ofstream outFile = openOutputOrThrow(outputFile);
        // outFile << "/** \n\n *time (sec), prn, phase bias \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<PhaseBias> result_bias = results.filter<PhaseBias>();
        for (const auto& key_value : result_bias)
        {
                int index = epoch++;
                checkIndexSize(static_cast<std::size_t>(index), satIndex.size(), "satIndex");
                PhaseBias p = key_value.value;
                outFile << satIndex[index] <<  " " << p.value() << std::endl;

        }
}

}
