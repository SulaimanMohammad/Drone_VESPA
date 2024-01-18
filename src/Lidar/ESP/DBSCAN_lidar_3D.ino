#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <ESP32Servo.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <map>
#include <numeric>
#include <deque> // For the sliding window

#define DEV_I2C Wire
#define SerialPort Serial

#define argent_warning_distance 40
bool warning_distance = false;
int points_in_warning_distance = 0;
/* ---------- Lidar variables ---------- */
const int numSamples = 1; // number of samples for one reading
#define FOV 3             // Field of View of the sensor
#define pan_start_range 20
#define pan_end_range 120
#define tilt_start_range 40
#define tilt_end_range 120

#define object_FOV (FOV * 3) // Field of object detection (enough angles to detect object)
#define pan_active_range (pan_end_range - pan_start_range)
#define tilt_active_range (tilt_end_range - tilt_start_range)
#define MAX_POINTS (((pan_active_range / object_FOV) + 1) * ((tilt_active_range / object_FOV) + 1))
#define Sensor_max_range 6000 // it is maximum reading the sensor give here 6m in mm
#define Sensor_min_range 20
#define Limit_distance Sensor_max_range + (Sensor_max_range * 0.20)
/* ---------- Servo variables ---------- */
#define pan_controle 2       // Pin on the bord for control signal
#define tilt_controle 4      // Pin on the bord for control signal
#define servo_1degree_time 6 // milliseconds

/* ---------- DBSCAN parameters ---------- */
#define minPoints 3               // Number of close point to form cluster
#define angleWeight 0.1           // How imortant is the connectivity of the angle with distance
#define shift_margin_to_merge 300 // Max diffeence in distance between tow clusters core to merge them
#define NOISE -2
#define UNCLASSIFIED -1
int numPoints = MAX_POINTS;
float thresholdX = 50;
float thresholdY = 50;
float thresholdZ = 50;

struct ClusterInfo
{
    float sumX, sumY, sumZ;                // Coordinates for centroid
    float centroidX, centroidY, centroidZ; // Centroid coordinates
    float centerDistance;                  // DIstance of center of cluster from sensor
    int count;                             // Count of points in the cluster
    int corePointIndex;                    // Index of the core point
    float corePointDistance;               // Distance of the core point
    int corePointNumNeighbors;             // Number of neighbors for the core point
    float minDistance;                     // Minimum distance in the cluster
    int minDistancePointIndex;             // Index of the point with minimum distance
    int sweep;                             // 1 for first sweep (0-180), 2 for second sweep (180-0)
    float topsisScore;                     // scores based on criteria
    int sweep1Index;                       // Index of the cluster in clustersSweep1
    int sweep2Index;                       // Index of the cluster in clustersSweep2
};
int numberOfClusters;
/* ---------- TOPSIS parameters ---------- */
#define weightCorePointDistance 0.31
#define weightNumberOfPoints 0.09
#define weightMinDistance 0.1
#define weightCenterDistance 0.5

// First merge between 2 sweeps
#define scoreMergeThreshold 0.05 // Define a threshold for merging clusters based on TOPSIS score similarity
#define distanceMergeThreshold 200
// Second merge of the result of first one for general view of objects
#define PostMergeTOPSIS_scoreThreshold 0.02
#define PostMergeTOPSIS_distanceThreshold 100
float globalMaxCorePointDistance = FLT_MIN;
float globalMaxNumberOfPoints = FLT_MIN;
float globalMaxCenterDistance = FLT_MIN;
float globalMaxMinDistance = FLT_MIN;

/* ----------  K-distance to define espsilon of DBSCAN parameters ---------- */
const int K = minPoints;
float coeff_elbow = 0;
float coeff_Knee = 1 - coeff_elbow;
struct KDistance
{
    float distance;
    int index;
};

/*--- Point represent connection between distance-angle in 2D plan --*/
struct Point
{
    float x, y, z;
    int distance = Limit_distance;
    float pan_angle, tilt_angle;
    bool visited = false;
    int clusterId = UNCLASSIFIED;
};
Point points[MAX_POINTS];
std::map<int, std::vector<Point>> clusterMap;

/* ----------  Initialization ---------- */
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);
Servo pan_servo;
Servo tilt_servo;

/*----------- Functions Declaration -------*/
void move_collect_data(int start_point, int end_point);                              // Move servo and save distance for each angle
void detect_objects_clustering(std::vector<ClusterInfo> &clusters);                  // Call DBSCAN, calcul esp and count number of objects
void resetData();                                                                    // Reset data after sweeps 0-180 to 180-0
void print_position_distance(int arrayIndex, int servoPosition, int distance_value); // print data of each angle and distance
void reorderPointsForConsistentProcessing();                                         // Reorder Point to match 0-180 with 180 to 0
Point sphericalToCartesian(int pan_angle, int tilt_angle, float distance);           // Convert point from ( angle, distance) to (x,y)
float calculateDistance(const Point &p1, const Point &p2);                           // Calculate the distance between 2 points considering the distance and the angle
void DBSCAN();
std::vector<ClusterInfo> mergeClustersBasedOnTopsis(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2);
void calculateKDistance_set_Epsilon();
void print_kDistance(float kDistances[], int numPoints);
void gather_clusters_info(std::vector<ClusterInfo> &clusters);
void printClustersInfo(const std::vector<ClusterInfo> &clusters, int sweep);
int get_distance();
void reportMemory();

void setup()
{
    SerialPort.begin(115200); // Initialize serial for output.
    SerialPort.println("Starting...");
    DEV_I2C.begin();                                 // Initialize I2C bus.
    sensor_vl53l4cx_sat.begin();                     // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();              // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.InitSensor(0x12);            // Initialize VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement(); // Start Measurements

    // Set servo
    pan_servo.setPeriodHertz(50);
    tilt_servo.setPeriodHertz(50);
    pan_servo.attach(pan_controle);
    tilt_servo.attach(tilt_controle);
    pan_servo.write(0);
    tilt_servo.write(0);
}

void loop()
{
    Serial.println(" Sweep 0-180");
    move_collect_data(pan_start_range, pan_end_range);
    std::vector<ClusterInfo> clustersSweep1;
    detect_objects_clustering(clustersSweep1);
    printClustersInfo(clustersSweep1, 1);

    Serial.println(" Sweep 180-0");
    move_collect_data(pan_end_range, pan_start_range);
    reorderPointsForConsistentProcessing(); // Reorder points for consistent processing

    std::vector<ClusterInfo> clustersSweep2;
    detect_objects_clustering(clustersSweep2);
    printClustersInfo(clustersSweep2, 2);

    std::vector<ClusterInfo> mergedClusters = mergeClustersBasedOnTopsis(clustersSweep1, clustersSweep2);
    Serial.println("            Merged Cluster Info: ");
    for (const auto &cluster : mergedClusters)
    {
        Serial.print("CorePointDistance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" \tMin Distance: ");
        Serial.println(cluster.minDistance);
    }
    resetData();
    delay(500); // Wait for stabilization
    Serial.println("-------------------------------------------------");
    // All vectors sweep1,sweep2 and merged cluster are local of each iteration, at end of each iter they will be destroyed and mem free
}

/*
------------------------------------------------------------
-------- Move Servos & collect_data functions --------------
------------------------------------------------------------
*/
void move_servo_to_position(Servo &servo, int position)
{
    servo.write(position);
    delay(servo_1degree_time * object_FOV); // Wait for stabilization
}

Point update_point(int pan_servoPosition, int tilt_servoPosition, int distance, int index)
{
    if (distance > Sensor_min_range)
    {
        if (points[index].distance == Limit_distance)
        {
            return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, distance);
        }
        else
        {
            return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, (points[index].distance + distance) / 2.0);
        }
    }
    return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, Limit_distance);
}

int calculate_index(int pan_servoPosition, int tilt_servoPosition)
{
    // Determine the pan and tilt steps based on the current angle and the object FOV
    int panIndex = (abs(pan_servoPosition - pan_start_range) / object_FOV);
    int tiltIndex = (abs(tilt_servoPosition - tilt_start_range) / object_FOV);

    // Calculate the total number of tilt steps to determine how tilt affects the index
    int totalTiltSteps = (abs(tilt_end_range - tilt_start_range) / object_FOV) + 1;

    // The index is then the combination of pan and tilt steps
    return panIndex * totalTiltSteps + tiltIndex;
}

void calculate_tilt_range(int &tiltStart, int &tiltEnd, int &tiltStep, int pan_servoPosition, bool reverse)
{
    bool isEvenPanStep = (pan_servoPosition / object_FOV) % 2 == 0;

    if (reverse && pan_servoPosition == pan_end_range)
    {
        tiltStart = isEvenPanStep ? tilt_end_range : tilt_start_range; // Start from the highest tilt angle
        tiltEnd = isEvenPanStep ? tilt_start_range : tilt_end_range;   // Move to the lowest tilt angle
        isEvenPanStep = false;
    }
    else
    {
        tiltStart = isEvenPanStep ? tilt_start_range : tilt_end_range;
        tiltEnd = isEvenPanStep ? tilt_end_range : tilt_start_range;
    }

    tiltStep = (tiltStart < tiltEnd) ? object_FOV : -object_FOV;
}

void print_point_data(int index, int pan_servoPosition, int tilt_servoPosition, int distance)
{
    Serial.print("point index : ");
    Serial.print(index);
    Serial.print(", Pan Angle: ");
    Serial.print(pan_servoPosition);
    Serial.print(", Tilt Angle: ");
    Serial.print(tilt_servoPosition);
    Serial.print(", read Distance: ");
    Serial.print(distance);
    Serial.print(", Point: (");
    Serial.print(points[index].x);
    Serial.print(", ");
    Serial.print(points[index].y);
    Serial.print(", ");
    Serial.print(points[index].z);
    Serial.println(")");
}
void move_collect_data(int start_point, int end_point)
{
    int index = 0;
    int step = (start_point < end_point) ? object_FOV : -object_FOV;
    bool reverse = (start_point < end_point) ? false : true; // Determine the direction based on start and end values
    int tiltStart, tiltEnd, tiltStep;
    for (int pan_servoPosition = start_point; (step > 0) ? (pan_servoPosition <= end_point) : (pan_servoPosition >= end_point); pan_servoPosition += step)
    {
        bool isEvenPanStep = (pan_servoPosition / object_FOV) % 2 == 0;

        move_servo_to_position(pan_servo, pan_servoPosition);
        calculate_tilt_range(tiltStart, tiltEnd, tiltStep, pan_servoPosition, reverse);

        for (int tilt_servoPosition = tiltStart; isEvenPanStep ? (tilt_servoPosition <= tiltEnd) : (tilt_servoPosition >= tiltEnd); tilt_servoPosition += tiltStep)
        {
            move_servo_to_position(tilt_servo, tilt_servoPosition);
            index = calculate_index(pan_servoPosition, tilt_servoPosition);
            int distance = get_distance();
            points[index] = update_point(pan_servoPosition, tilt_servoPosition, distance, index);
            print_point_data(index, pan_servoPosition, tilt_servoPosition, distance);

            if (distance > Sensor_min_range && distance <= argent_warning_distance)
            {
                points_in_warning_distance++;
                if (points_in_warning_distance > 2)
                {
                    Serial.println(" EMERGENCY ");
                    points_in_warning_distance = 0;
                }
            }
        }
        if (pan_servoPosition == end_point)
        {
            break;
        }
    }
}

/*
------------------------------------------------------------
------------------Set the point in 2D plan -----------------
------------------------------------------------------------
*/
Point sphericalToCartesian(int pan_angle, int tilt_angle, float distance)
{
    Point p;
    p.distance = distance;
    p.pan_angle = pan_angle;
    p.tilt_angle = tilt_angle;
    // Convert angles from degrees to radians
    float theta = static_cast<float>(pan_angle) * PI / 180.0f; // Pan angle in radians
    float phi = static_cast<float>(tilt_angle) * PI / 180.0f;  // Tilt angle in radians

    // Convert to Cartesian coordinates
    p.x = distance * cos(theta) * sin(phi);
    p.y = distance * sin(theta) * sin(phi);
    p.z = distance * cos(phi);

    return p;
}

// Function to calculate Euclidean distance between two points with angle consideration
float calculateDistance(const Point &p1, const Point &p2)
{
    // Calculate the Euclidean distance in 3D
    float spatialDist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    return spatialDist;
}

void resetData()
{

    // Reset the points array
    for (int i = 0; i < MAX_POINTS; i++)
    {
        points[i] = Point(); // Reset each point (assuming default constructor sets it to a default state)
        points[i].clusterId = UNCLASSIFIED;
        points[i].visited = false;
    }
}
void print_position_distance(int arrayIndex, int servoPosition, int distance_value)
{
    Serial.print("arrayIndex");
    Serial.print(arrayIndex);
    SerialPort.print(" ,Position: ");
    SerialPort.print(servoPosition);
    SerialPort.print(", Updated Average: ");
    SerialPort.println(distance_value);
}

void reorderPointsForConsistentProcessing()
{
    int i = 0;
    int j = numPoints - 1;
    while (i < j)
    {
        // Swap points[i] and points[j]
        Point temp = points[i];
        points[i] = points[j];
        points[j] = temp;
        i++;
        j--;
    }
}

/*
-------------------------------------------------------------
--------------------------- DBSCAN --------------------------
-------------------------------------------------------------
*/
void detect_objects_clustering(std::vector<ClusterInfo> &clusters)
{
    // Calculate k-distance for each point, Calculate espilon
    calculateKDistance_set_Epsilon();

    if (numPoints > 0)
    {
        // Apply DBSCAN for clustering only on valid points
        DBSCAN();
    }
    else
    {
        Serial.println("No valid data points collected.");
    }

    // Clear previous cluster data
    clusterMap.clear();

    // Populate cluster map with points
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0)
        { // Assuming clusterId <= 0 are noise or unclassified
            clusterMap[clusterId].push_back(points[i]);
        }
    }

    gather_clusters_info(clusters); // Save the info of clusters in vector
}

bool arePointsNeighbors(const Point &p1, const Point &p2)
{

    return abs(p1.x - p2.x) <= thresholdX && abs(p1.y - p2.y) <= thresholdY && abs(p1.z - p2.z) <= thresholdZ;
}

void getNeighbors(int pointIndex, std::vector<int> &neighbors)
{
    neighbors.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        if (arePointsNeighbors(points[pointIndex], points[i]))
        {
            neighbors.push_back(i);
        }
    }
}

void expandCluster(int pointIndex, int clusterId, std::vector<int> &neighbors)
{
    points[pointIndex].clusterId = clusterId;

    for (int i = 0; i < neighbors.size(); ++i)
    {
        int currNeighbor = neighbors[i];
        if (points[currNeighbor].clusterId == UNCLASSIFIED || points[currNeighbor].clusterId == NOISE)
        {
            points[currNeighbor].clusterId = clusterId;

            std::vector<int> newNeighbors;
            getNeighbors(currNeighbor, newNeighbors);

            for (int newNeighbor : newNeighbors)
            {
                // Check if new neighbor is already in neighbors vector
                if (std::find(neighbors.begin(), neighbors.end(), newNeighbor) == neighbors.end())
                {
                    neighbors.push_back(newNeighbor);
                }
            }
        }
    }
}

void DBSCAN()
{
    int clusterId = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        if (points[i].distance == 0 || points[i].distance == Limit_distance)
        {
            continue;
        }
        if (points[i].clusterId == UNCLASSIFIED)
        {
            std::vector<int> neighbors;
            getNeighbors(i, neighbors);
            if (neighbors.size() < minPoints)
            {
                points[i].clusterId = NOISE;
            }
            else
            {
                expandCluster(i, ++clusterId, neighbors);
            }
        }
    }
}

/*
-------------------------------------------------------------
--------------------- Merge clusters ------------------------
-------------------------------------------------------------
*/
// Function to normalize the cluster data
void normalizeClustersData(std::vector<ClusterInfo> &clusters)
{
    // Find the maximum values
    for (const auto &cluster : clusters)
    {
        globalMaxCorePointDistance = std::max(globalMaxCorePointDistance, cluster.corePointDistance);
        globalMaxNumberOfPoints = std::max(globalMaxNumberOfPoints, float(cluster.count));
        globalMaxMinDistance = std::max(globalMaxMinDistance, cluster.minDistance);
        globalMaxCenterDistance = std::max(globalMaxCenterDistance, cluster.centerDistance);
    }
    // Normalize the data
    for (auto &cluster : clusters)
    {
        cluster.corePointDistance /= globalMaxCorePointDistance;
        cluster.count = float(cluster.count) / globalMaxNumberOfPoints;
        cluster.minDistance /= globalMaxMinDistance;
        cluster.centerDistance /= globalMaxCenterDistance;
    }
}

void Reverse_normalization(ClusterInfo &cluster)
{
    cluster.corePointDistance *= globalMaxCorePointDistance;
    cluster.count *= globalMaxNumberOfPoints;
    cluster.minDistance *= globalMaxMinDistance;
    cluster.centerDistance *= globalMaxCenterDistance;
}

// Function to calculate TOPSIS score for each cluster
void calculateTopsisScores(std::vector<ClusterInfo> &clusters)
{

    // Normalize the cluster data
    normalizeClustersData(clusters);

    // Calculate the ideal and negative-ideal solutions
    float idealCorePointDistance = 1, idealNumberOfPoints = 1, idealMinDistance = 1;
    float negIdealCorePointDistance = 0, negIdealNumberOfPoints = 0, negIdealMinDistance = 0;
    float idealCenterDistance = 1;
    float negIdealCenterDistance = 0;
    // Calculate the separation measures and the relative closeness
    for (auto &cluster : clusters)
    {
        float distanceToIdeal = sqrt(
            pow(weightCorePointDistance * (idealCorePointDistance - cluster.corePointDistance), 2) +
            pow(weightNumberOfPoints * (idealNumberOfPoints - cluster.count), 2) +
            pow(weightMinDistance * (idealMinDistance - cluster.minDistance), 2) +
            pow(weightCenterDistance * (idealCenterDistance - cluster.centerDistance), 2) // Include centerDistance
        );

        float distanceToNegIdeal = sqrt(
            pow(weightCorePointDistance * (cluster.corePointDistance - negIdealCorePointDistance), 2) +
            pow(weightNumberOfPoints * (cluster.count - negIdealNumberOfPoints), 2) +
            pow(weightMinDistance * (cluster.minDistance - negIdealMinDistance), 2) +
            pow(weightCenterDistance * (cluster.centerDistance - negIdealCenterDistance), 2) // Include centerDistance
        );

        float denominator = distanceToNegIdeal + distanceToIdeal;
        if (denominator == 0)
        {
            Serial.println("Warning: Denominator is zero in TOPSIS score calculation");
        }

        cluster.topsisScore = distanceToNegIdeal / (distanceToNegIdeal + distanceToIdeal);
        Reverse_normalization(cluster);
    }
}

ClusterInfo mergeTwoClusters(const ClusterInfo &c1, const ClusterInfo &c2)
{
    ClusterInfo mergedCluster;
    mergedCluster.sumX = (c1.sumX + c2.sumX) / 2.0;
    mergedCluster.sumY = (c1.sumY + c2.sumY) / 2.0;
    mergedCluster.count = (c1.count + c2.count) / 2.0;
    mergedCluster.corePointDistance = std::min(c1.corePointDistance, c2.corePointDistance);
    mergedCluster.minDistance = std::min(c1.minDistance, c2.minDistance);
    return mergedCluster;
}

std::vector<ClusterInfo> mergeClustersPostTopsis(std::vector<ClusterInfo> &clusters)
{
    if (clusters.size() <= 1)
    {
        return clusters; // No merging needed if there's one or no clusters
    }

    calculateTopsisScores(clusters); // Calculate TOPSIS scores for each cluster

    bool mergeHappened;

    do
    {
        mergeHappened = false;
        std::vector<ClusterInfo> newClusters;
        std::vector<bool> merged(clusters.size(), false);

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (!merged[i])
            {
                ClusterInfo bestMergeCluster = clusters[i];
                int bestMergePartner = -1;
                float bestMergeScore = FLT_MAX;

                for (size_t j = i + 1; j < clusters.size(); ++j)
                {
                    if (!merged[j])
                    {
                        float scoreDiff = std::abs(clusters[i].topsisScore - clusters[j].topsisScore);
                        float distanceDiff = std::abs(clusters[i].minDistance - clusters[j].minDistance);

                        if (scoreDiff < PostMergeTOPSIS_scoreThreshold && distanceDiff < PostMergeTOPSIS_distanceThreshold && scoreDiff < bestMergeScore)
                        {
                            bestMergeScore = scoreDiff;
                            bestMergePartner = j;
                        }
                    }
                }

                if (bestMergePartner != -1)
                {
                    bestMergeCluster = mergeTwoClusters(bestMergeCluster, clusters[bestMergePartner]); // Assumes a function to merge two clusters
                    merged[bestMergePartner] = true;
                    mergeHappened = true;
                }

                newClusters.push_back(bestMergeCluster);
            }
        }

        clusters = newClusters;
    } while (mergeHappened);

    return clusters;
}
std::vector<ClusterInfo> mergeClustersBasedOnTopsis(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2)
{
    // Calculate scores for all clusters
    calculateTopsisScores(clustersSweep1);
    calculateTopsisScores(clustersSweep2);

    std::vector<ClusterInfo> mergedClusters;
    std::vector<bool> mergedSweep1(clustersSweep1.size(), false);
    std::vector<bool> mergedSweep2(clustersSweep2.size(), false);

    for (size_t cluster1Index = 0; cluster1Index < clustersSweep1.size(); ++cluster1Index)
    {
        const auto &cluster1 = clustersSweep1[cluster1Index];
        float minDistanceDiff = Limit_distance;
        float closestScoreDiff = FLT_MAX;
        int closestClusterIndex = -1;

        // Find the closest cluster from sweep 2 based on TOPSIS score
        for (size_t i = 0; i < clustersSweep2.size(); ++i)
        {
            float scoreDiff = std::abs(cluster1.topsisScore - clustersSweep2[i].topsisScore);
            float distanceDiff = abs(cluster1.minDistance - clustersSweep2[i].minDistance);

            // Debugging: Print the TOPSIS score comparison
            /*
            Serial.print("Comparing Sweep 1 Cluster ");
            Serial.print(cluster1Index);
            Serial.print(" (Score: ");
            Serial.print(cluster1.topsisScore);
            Serial.print(") with Sweep 2 Cluster ");
            Serial.print(i);
            Serial.print(" (Score: ");
            Serial.print(clustersSweep2[i].topsisScore);
            Serial.print("), Score Difference: ");
            Serial.print(scoreDiff);
            Serial.print(" ,distanceDiff");
            Serial.println(distanceDiff);
            */

            if (distanceDiff < distanceMergeThreshold && scoreDiff < closestScoreDiff)
            {
                closestScoreDiff = scoreDiff;
                minDistanceDiff = distanceDiff;
                closestClusterIndex = i;
            }
        }

        // Merge the closest clusters if within the threshold
        if (closestClusterIndex != -1 && closestScoreDiff < scoreMergeThreshold)
        {
            ClusterInfo mergedCluster = mergeTwoClusters(cluster1, clustersSweep2[closestClusterIndex]);
            mergedCluster.sweep1Index = cluster1Index;
            mergedCluster.sweep2Index = closestClusterIndex;
            mergedClusters.push_back(mergedCluster);
            mergedSweep1[cluster1Index] = true;
            mergedSweep2[closestClusterIndex] = true;

            // Debugging: Print info about the merged clusters
            /*
            Serial.print("Merged Sweep 1 Cluster ");
            Serial.print(cluster1Index);
            Serial.print(" with Sweep 2 Cluster ");
            Serial.println(closestClusterIndex);
            */
        }
        else
        {
            ClusterInfo unmergedCluster = cluster1;
            unmergedCluster.sweep1Index = cluster1Index;
            unmergedCluster.sweep2Index = -1;
            mergedClusters.push_back(unmergedCluster);

            // Debugging: Print info about the unmerged cluster

            /*
            Serial.print("No suitable merge for Sweep 1 Cluster ");
            Serial.println(cluster1Index);
            */
        }
    }

    for (size_t i = 0; i < clustersSweep2.size(); ++i)
    {
        if (!mergedSweep2[i])
        {
            mergedClusters.push_back(clustersSweep2[i]);
        }
    }

    Serial.println(" First Merged Cluster Info: ");
    for (const auto &cluster : mergedClusters)
    {
        Serial.print("CorePointDistance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" \tMin Distance: ");
        Serial.println(cluster.minDistance);
    }

    return mergeClustersPostTopsis(mergedClusters);
}

/*
---------------------------------------------------------------
---------------------- Find epsilon od DBSCAN -----------------
---------------------------------------------------------------
*/
// Elbow Method
float findElbowPoint(const std::vector<float> &kDistances)
{
    int numPoints = kDistances.size();
    std::vector<int> elbowIndices; // Store indices of all elbow points

    for (int i = 1; i < numPoints - 1; ++i)
    {
        // Finding local maxima in the k-distance graph
        if (kDistances[i] > kDistances[i - 1] && kDistances[i] > kDistances[i + 1])
        {
            elbowIndices.push_back(i);
        }
    }

    // Calculate the average of all elbow points
    float sumElbowDistances = 0;
    for (int index : elbowIndices)
    {
        sumElbowDistances += kDistances[index];
    }

    float averageElbowDistance = 0;
    if (!elbowIndices.empty())
    {
        averageElbowDistance = sumElbowDistances / elbowIndices.size();
    }

    return averageElbowDistance;
}

float findKneePoint_simple(const std::vector<float> &kDistances)
{
    int numPoints = kDistances.size();
    std::vector<int> kneeIndices; // Store indices of all knee points

    for (int i = 1; i < numPoints - 1; ++i)
    {
        // Finding points with significant change in slope
        float diff1 = kDistances[i] - kDistances[i - 1];
        float diff2 = kDistances[i + 1] - kDistances[i];
        if (diff1 * diff2 < 0)
        { // Change in the sign of the slope
            kneeIndices.push_back(i);
        }
    }

    // Calculate the average of all knee points
    float sumKneeDistances = 0;
    for (int index : kneeIndices)
    {
        sumKneeDistances += kDistances[index];
    }

    float averageKneeDistance = 0;
    if (!kneeIndices.empty())
    {
        averageKneeDistance = sumKneeDistances / kneeIndices.size();
    }

    return averageKneeDistance;
}

// this knee finder use Derivative-Based Method , Significant Knee Selection, Outlier Handling

float findKneePoint(const std::vector<float> &kDistances)
{
    int numPoints = kDistances.size();
    if (numPoints < 3)
    {
        return -1; // Not enough points to determine a knee
    }

    // Function to calculate the angle between three points
    auto angle = [](float a, float b, float c) -> float
    {
        float angle = atan2(c - b, 1.0f) - atan2(b - a, 1.0f);
        return fabs(angle);
    };

    float maxAngle = 0.0;
    int kneeIndex = -1;

    // Iterate over the points to find the maximum angle
    for (int i = 1; i < numPoints - 1; ++i)
    {
        float currentAngle = angle(kDistances[i - 1], kDistances[i], kDistances[i + 1]);
        if (currentAngle > maxAngle)
        {
            maxAngle = currentAngle;
            kneeIndex = i;
        }
    }

    return (kneeIndex != -1) ? kDistances[kneeIndex] : -1;
}

int estimateNoisePoints(float thresholdX, float thresholdY, float thresholdZ)
{
    int noisePointCount = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        int neighborCount = 0;
        for (int j = 0; j < numPoints; ++j)
        {
            if (i != j &&
                abs(points[i].x - points[j].x) <= thresholdX &&
                abs(points[i].y - points[j].y) <= thresholdY &&
                abs(points[i].z - points[j].z) <= thresholdZ)
            {
                neighborCount++;
            }
        }
        if (neighborCount < minPoints)
        {
            noisePointCount++;
        }
    }
    return noisePointCount;
}

void adjustThresholdsBasedOnFeedback(float elbowValueX, float kneeValueX,
                                     float elbowValueY, float kneeValueY,
                                     float elbowValueZ, float kneeValueZ)
{
    // Find the combination that produces the minimum noise for each dimension
    std::vector<float> thresholdCandidatesX = {elbowValueX, kneeValueX};
    std::vector<float> thresholdCandidatesY = {elbowValueY, kneeValueY};
    std::vector<float> thresholdCandidatesZ = {elbowValueZ, kneeValueZ};

    float minNoiseX = std::numeric_limits<float>::max();
    float minNoiseY = std::numeric_limits<float>::max();
    float minNoiseZ = std::numeric_limits<float>::max();

    // Optimal thresholds
    float optimalThresholdX = thresholdCandidatesX[0];
    float optimalThresholdY = thresholdCandidatesY[0];
    float optimalThresholdZ = thresholdCandidatesZ[0];

    for (float candidateX : thresholdCandidatesX)
    {
        for (float candidateY : thresholdCandidatesY)
        {
            for (float candidateZ : thresholdCandidatesZ)
            {
                int noisePoints = estimateNoisePoints(candidateX, candidateY, candidateZ);

                // Check for X
                if (candidateX == candidateY && candidateX == candidateZ && noisePoints < minNoiseX)
                {
                    minNoiseX = noisePoints;
                    optimalThresholdX = candidateX;
                }

                // Check for Y
                if (candidateY == candidateX && candidateY == candidateZ && noisePoints < minNoiseY)
                {
                    minNoiseY = noisePoints;
                    optimalThresholdY = candidateY;
                }

                // Check for Z
                if (candidateZ == candidateX && candidateZ == candidateY && noisePoints < minNoiseZ)
                {
                    minNoiseZ = noisePoints;
                    optimalThresholdZ = candidateZ;
                }
            }
        }
    }

    thresholdX = optimalThresholdX;
    thresholdY = optimalThresholdY;
    thresholdZ = optimalThresholdZ;

    Serial.print("Optimal Threshold X: ");
    Serial.println(thresholdX);
    Serial.print("Optimal Threshold Y: ");
    Serial.println(thresholdY);
    Serial.print("Optimal Threshold Z: ");
    Serial.println(thresholdZ);
}

/*
---------------------------------------------------------------
------------- K-Distance to define epsilon od DBSCAN ----------
---------------------------------------------------------------
*/

// Function to calculate the k-distance for each point
void calculateKDistance_set_Epsilon()
{
    std::vector<float> kDistancesX(numPoints);
    std::vector<float> kDistancesY(numPoints);
    std::vector<float> kDistancesZ(numPoints);

    for (int i = 0; i < numPoints; ++i)
    {
        // Store distances to all other points in each dimension
        std::vector<float> distancesX, distancesY, distancesZ;
        for (int j = 0; j < numPoints; ++j)
        {
            if (i != j)
            {
                distancesX.push_back(abs(points[i].x - points[j].x));
                distancesY.push_back(abs(points[i].y - points[j].y));
                distancesZ.push_back(abs(points[i].z - points[j].z));
            }
        }

        // Sort and pick the Kth distance
        std::sort(distancesX.begin(), distancesX.end());
        std::sort(distancesY.begin(), distancesY.end());
        std::sort(distancesZ.begin(), distancesZ.end());

        kDistancesX[i] = distancesX[K - 1];
        kDistancesY[i] = distancesY[K - 1];
        kDistancesZ[i] = distancesZ[K - 1];
    }

    // Calculate elbow and knee points for each dimension
    float elbowPointX = findElbowPoint(kDistancesX);
    float kneePointX = findKneePoint(kDistancesX);

    float elbowPointY = findElbowPoint(kDistancesY);
    float kneePointY = findKneePoint(kDistancesY);

    float elbowPointZ = findElbowPoint(kDistancesZ);
    float kneePointZ = findKneePoint(kDistancesZ);

    // Print the calculated points
    Serial.print("Elbow Point X: ");
    Serial.println(elbowPointX);
    Serial.print("Knee Point X: ");
    Serial.println(kneePointX);

    Serial.print("Elbow Point Y: ");
    Serial.println(elbowPointY);
    Serial.print("Knee Point Y: ");
    Serial.println(kneePointY);

    Serial.print("Elbow Point Z: ");
    Serial.println(elbowPointZ);
    Serial.print("Knee Point Z: ");
    Serial.println(kneePointZ);

    // Call the function with the calculated values
    adjustThresholdsBasedOnFeedback(elbowPointX, kneePointX,
                                    elbowPointY, kneePointY,
                                    elbowPointZ, kneePointZ);
}

void print_kDistance(float kDistances[], int numPoints)
{
    for (int i = 0; i < numPoints; ++i)
    {
        if (kDistances[i] != -1)
        {
            Serial.print("Point ");
            Serial.print(i);
            Serial.print(" k-distance: ");
            Serial.println(kDistances[i]);
        }
    }
}

/*
----------------------------------------------------------------
---------------------- Info of clusters ------------------------
----------------------------------------------------------------
*/

int countUniqueClusters(Point points[], int numPoints)
{
    std::vector<bool> uniqueClusters;
    int numUniqueClusters = 0;

    for (int i = 0; i < numPoints; i++)
    {
        if (points[i].clusterId > 0)
        {
            // Resize vector if needed
            if (points[i].clusterId >= uniqueClusters.size())
            {
                uniqueClusters.resize(points[i].clusterId + 1, false);
            }

            // Check and mark this cluster ID as found
            if (!uniqueClusters[points[i].clusterId])
            {
                uniqueClusters[points[i].clusterId] = true;
                numUniqueClusters++;
            }
        }
    }

    return numUniqueClusters + 1;
}

void gather_clusters_info(std::vector<ClusterInfo> &clusters)
{
    numberOfClusters = countUniqueClusters(points, numPoints); // Get the number of clusters
    Serial.print(" Number of objects : ");
    Serial.println(numberOfClusters - 1);
    ClusterInfo clusterData[numberOfClusters];

    // Initialize cluster data
    for (int i = 0; i < numberOfClusters; ++i)
    {
        clusterData[i] = {0, 0, 0,
                          0, 0, 0,
                          0,
                          0,
                          -1, 0, 0,
                          Limit_distance, -1, 0, 0, 0, 0}; // Initialize minDistance with FLT_MAX
    }

    // Iterating through each point
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0 && clusterId <= numberOfClusters)
        {
            // Accumulate for centroid calculation
            clusterData[clusterId].sumX += points[i].x;
            clusterData[clusterId].sumY += points[i].y;
            clusterData[clusterId].sumZ += points[i].z;
            clusterData[clusterId].count++;
            int pointDistance = points[i].distance;

            // Check for minimum distance
            if (pointDistance < clusterData[clusterId].minDistance && pointDistance != 0)
            {
                clusterData[clusterId].minDistance = pointDistance;
                clusterData[clusterId].minDistancePointIndex = i;
            }
            // Check for core point (based on number of neighbors)
            std::vector<int> neighbors;
            getNeighbors(i, neighbors);
            int numNeighbors_core = neighbors.size();
            if (numNeighbors_core >= minPoints)
            {
                int currentDistance = points[i].distance;
                // Validate the current point's distance
                if (currentDistance > 0 && currentDistance < Limit_distance)
                {
                    // Assuming the core point is the one with the most neighbors
                    if (clusterData[clusterId].corePointIndex == -1 || numNeighbors_core > clusterData[clusterId].corePointNumNeighbors)
                    {
                        clusterData[clusterId].corePointIndex = i;
                        clusterData[clusterId].corePointNumNeighbors = numNeighbors_core;

                        int corePointIndex = clusterData[clusterId].corePointIndex;
                        if (corePointIndex >= 0 && corePointIndex < numPoints)
                        {

                            clusterData[clusterId].corePointDistance = points[corePointIndex].distance; // Access the distance using flatIndex
                        }
                    }
                }
            }
        }
    }

    // Populate the clusters vector
    for (int i = 0; i < numberOfClusters; ++i)
    {
        if (clusterData[i].count > 0)
        {
            clusterData[i].centroidX = clusterData[i].sumX / clusterData[i].count;
            clusterData[i].centroidY = clusterData[i].sumY / clusterData[i].count;
            clusterData[i].centroidZ = clusterData[i].sumZ / clusterData[i].count;
            clusterData[i].centerDistance = sqrt(pow(clusterData[i].centroidX, 2) + pow(clusterData[i].centroidY, 2) + pow(clusterData[i].centroidZ, 2));
            clusters.push_back(clusterData[i]);
        }
    }
}

void printClustersInfo(const std::vector<ClusterInfo> &clusters, int sweep)
{
    for (const auto &cluster : clusters)
    {
        Serial.print("Cluster Centroid: (");
        Serial.print(cluster.centroidX);
        Serial.print(", ");
        Serial.print(cluster.centroidY);
        Serial.print(", ");
        Serial.print(cluster.centroidZ);
        Serial.print(")");
        Serial.print(" , ");
        Serial.print("Distance: ");
        Serial.print(cluster.centerDistance);
        Serial.print(" , ");
        Serial.print("Number of Points: ");
        Serial.print(cluster.count);
        Serial.print(" , Core Point Num Neighbors: ");
        Serial.print(cluster.corePointNumNeighbors);
        Serial.print(" , Core Point distance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" , Minimum Distance: ");
        Serial.println(cluster.minDistance);
    }
    // New code to print all points in the cluster
    Serial.println("Points in this cluster:");
    for (const auto &pair : clusterMap)
    {
        int clusterId = pair.first;
        const auto &points = pair.second;

        Serial.print("Cluster ID: ");
        Serial.println(clusterId);

        for (const auto &point : points)
        {
            Serial.print("cluster indes : ");
            Serial.print(point.clusterId);
            Serial.print(", Coordinates: (");
            Serial.print(point.x);
            Serial.print(", ");
            Serial.print(point.y);
            Serial.print(", ");
            Serial.print(point.z);
            Serial.print(")");
            Serial.print(", Distance: ");
            Serial.print(point.distance);
            Serial.print(", pan angle: ");
            Serial.print(point.pan_angle);
            Serial.print(",  tilt angle: ");
            Serial.println(point.tilt_angle);
        }
    }
}

/*
-----------------------------------------------------------------
---------------------- Read data from sensor --------------------
-----------------------------------------------------------------
*/
// Function to calculate the median of an array of integers
int calculateMedianDistance(int samples[], int numSamples)
{
    // Sort the array of distances
    sortArray(samples, numSamples);

    // Compute the median
    if (numSamples % 2 != 0)
    { // Odd number of elements
        return samples[numSamples / 2];
    }
    else
    { // Even number of elementsa
        return (samples[numSamples / 2 - 1] + samples[numSamples / 2]) / 2;
    }
}

// Function to sort an array of integers
void sortArray(int arr[], int numElements)
{
    for (int i = 0; i < numElements - 1; i++)
    {
        for (int j = 0; j < numElements - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                // Swap arr[j] and arr[j+1]
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

int sensor_measurement()
{
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    int status;
    int chosenDistance = INT_MAX;
    int chosenStatus = -1;
    int measurement = 0;
    while (no_of_object_found == 0)
    {
        do
        {
            status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
        } while (!NewDataReady);

        if ((!status) && (NewDataReady != 0))
        {
            status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
            no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            for (j = 0; j < no_of_object_found; j++)
            {
                int currentDistance = pMultiRangingData->RangeData[j].RangeMilliMeter;
                int currentStatus = pMultiRangingData->RangeData[j].RangeStatus;
                // if ((currentStatus == 0 || currentStatus == 2 || currentStatus == 3 || currentStatus == 4 || currentStatus == 6 || currentStatus == 7) && currentDistance >= 0 && currentDistance < chosenDistance)
                if (currentDistance > 0 && currentDistance < chosenDistance)

                {
                    chosenDistance = currentDistance;
                    chosenStatus = currentStatus;
                }
            }

            if (chosenStatus != -1)
            {

                measurement = chosenDistance;
            }
            else
            {
                measurement = -1;
            }

            if (status == 0)
            {
                status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
            }
        }
    }
    return measurement;
}
int get_distance()
{
    int samples[numSamples];
    int validSamples = 0;
    long sumDistance = 0;
    int unvalid_counter = 0;
    for (int i = 0; i < numSamples; i++)
    {
        samples[i] = sensor_measurement();
        if (samples[i] < Sensor_max_range && samples[i] != -1)
        {
            sumDistance += samples[i];
            validSamples++;
        }
        else
        {
            unvalid_counter++;
        }
        delay(10);
    }

    if (unvalid_counter != numSamples)
    {
        int medianDistance = calculateMedianDistance(samples, numSamples);
        return medianDistance;
    }
    else
    {
        return -1;
    }
}

/*
---------------------------------------------
--------------ESP info-performance-----------
---------------------------------------------
*/
void reportMemory()
{
    // Report Free Heap Memory
    Serial.print("Free Heap Memory: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Largest Free Block: ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
