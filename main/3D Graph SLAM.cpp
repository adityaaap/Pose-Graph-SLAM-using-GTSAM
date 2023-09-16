#include <iostream>
//#include <gtsam/geometry/Rot2.h>
#include <sstream>
#include <string>
#include <fstream>
//#include <Eigen/Core>
#include <Eigen/Dense>
//#include <vector>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Quaternion.h>

using namespace std;
using namespace gtsam;

struct Pose{
    double id;
    double x;
    double y;
    double z;
    // double qx;
    // double qy;
    // double qz;
    // double qw;
    
    Eigen::Quaternion<double> q;
    Eigen::Matrix3d rot;
    Eigen::Matrix <double,1,3> translation;
    Pose(double id, double x,double y,double z, Eigen::Quaternion<double> q) : id(id), x(x), y(y), z(z), q(q) {
        rot = q.normalized().toRotationMatrix();
        translation << x,y,z;
    }
};

struct Edge{
    int i;
    int j;
    double x;
    double y;
    double z;
    Eigen::Quaternion<double> q;
    Eigen::Matrix3d rot;
    Eigen::Matrix <double,1,3> translation;
    Eigen::Matrix<double, 6, 6> cov;
    

    Edge(int i, int j, double x, double y, double z, Eigen::Quaternion<double> q, double arg[]): i(i), j(j), x(x), y(y), z(z), q(q) {
    
        rot = q.normalized().toRotationMatrix();
        translation << x,y,z;

        /* SETTING UP COVARIENCE MATRIX */
        cov(0,0) = arg[0];
        cov(0,1) = arg[1];
        cov(0,2) = arg[2];
        cov(0,3) = arg[3];
        cov(0,4) = arg[4];
        cov(0,5) = arg[5];

        cov(1,1) = arg[6];
        cov(1,2) = arg[7];
        cov(1,3) = arg[8];
        cov(1,4) = arg[9];
        cov(1,5) = arg[10];

        cov(2,2) = arg[11];
        cov(2,3) = arg[12];
        cov(2,4) = arg[13];
        cov(2,5) = arg[14];

        cov(3,3) = arg[15];
        cov(3,4) = arg[16];
        cov(3,5) = arg[17];

        cov(4,4) = arg[18];
        cov(4,5) = arg[19];

        /****/
        cov(5,5) = arg[20];
        /****/

        cov(1,0) = arg[1];
        cov(2,0) = arg[2];
        cov(3,0) = arg[3];
        cov(4,0) = arg[4];
        cov(5,0) = arg[5];

        cov(2,1) = arg[7];
        cov(3,1) = arg[8];
        cov(4,1) = arg[9];
        cov(5,1) = arg[10];

        cov(3,2) = arg[12];
        cov(4,2) = arg[13];
        cov(5,2) = arg[14];

        cov(4,3) = arg[16];
        cov(5,3) = arg[17];
        cov(5,4) = arg[19];

        /* CONVERTING INFO MATRIX TO COVARIENCE MATRIX */
        cov = cov.inverse().eval();
    };
};

pair<vector<Pose>, vector<Edge>> loadg2o(string path){
    std::ifstream infile(path);

    if(infile){
        std::cout<<"there"<<std::endl;
    } else{
        std::cout<<"There"<<std::endl;
    }

    /* POSE AND EDGE(constraints) DATA STORED */
    vector<Pose> data_pose;
    vector<Edge> data_edge;
    double i,j,k,l,qa,qb,qc,qd;
    double id1, id2, x, y, z, qx, qy, qz, qw, q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,q17,q18,q19,q20,q21;

    std::string line;

    while(infile.good()){
        infile >> line;
        if(line == "VERTEX_SE3:QUAT"){
            infile >> i >> j >> k >> l >> qa >> qb >> qc >> qd;
            Eigen::Quaternion<double> q = {qd, qa, qb, qc}; // FORMING QUATERNION FROM GIVEN DATA    
            data_pose.push_back(Pose(i,j,k,l,q));
        }
        else{
            if(line == "EDGE_SE3:QUAT"){
                infile >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw >> q1 >> q2 >> q3 >> q4 >> q5 >> q6 >> q7 >> q8 >> q9 >> q10 >> q11 >> q12 >> q13 >> q14 >> q15 >> q16 >> q17 >> q18 >> q19 >> q20 >> q21;
                Eigen::Quaternion<double> q_1 = {qw, qx, qy, qz};
                //double q_[] = {qw, qx, qy, qz};
                double arr[] = {q1, q2, q3, q4, q5, q6, q7, q8, q9 ,q10, q11, q12, q13, q14, q15, q16, q17, q18, q19, q20, q21};//ARRAY OF UPPER TRIANGULAR ELEMENTS
                data_edge.push_back(Edge(id1, id2, x, y, z, q_1, arr));
            }
        }
    }
    pair<vector<Pose>, vector<Edge>> result(data_pose, data_edge);
    return result;
}

void printEdge(vector<Edge> data_edge){
    cout<<"CHECK EDGE"<<endl;

    for(int i = 0; i<data_edge.size();i++){
        
        std::cout<<"Edge "<<i<<":"<<std::endl;
        std::cout<<"i: "<<data_edge[i].i<<std::endl;
        std::cout<<"j: "<<data_edge[i].j<<std::endl;
        std::cout.precision(7);
        std::cout<<"x: "<<data_edge[i].x<<std::endl;
        std::cout<<"y: "<<data_edge[i].y<<std::endl;
        std::cout<<"z: "<<data_edge[i].z<<std::endl;
        std::cout<<"quat: "<<data_edge[i].q<<std::endl;
        std::cout<<"rotation: "<<data_edge[i].rot<<std::endl;
        std::cout<<"translation: "<<data_edge[i].translation<<std::endl;
        std::cout<<"Cov: "<<data_edge[i].cov<<std::endl;
        std::cout<<std::endl;
    }
}
    
void printPose(vector<Pose> data_pose){
    cout<<"CHECK POSE"<<endl;
    for(int i=0; i<data_pose.size(); i++){
        cout<<"Pose "<<i<<endl;
        cout<<"id "<<data_pose[i].id<<endl;
        cout.precision(7);
        cout<<"x "<<data_pose[i].x<<endl;
        cout<<"y "<<data_pose[i].y<<endl;
        cout<<"z "<<data_pose[i].z<<endl;
        cout<<"quat "<<data_pose[i].q<<endl;
        cout<<"rotation "<<data_pose[i].rot<<endl;
        cout<<"translation "<<data_pose[i].translation<<endl;
        cout<<endl;
    }
}

gtsam::Values solution_Batch(gtsam::Values node, gtsam::NonlinearFactorGraph graph, vector<Pose> data_pose, vector<Edge> data_edge){
    /* INSERTING POSE DATA INTO NODES(initial values) */
    for(int i=0; i<data_pose.size(); i++){
        node.insert(data_pose[i].id, Pose3(Rot3(data_pose[i].rot), data_pose[i].translation));
    }
    //node.print("Nodes");
    //printNodes(node);

    /* ADDING PRIOR */
    gtsam::Pose3 priorMean(Rot3(data_pose[0].rot), data_pose[0].translation);
    auto prior_noise = gtsam::noiseModel::Gaussian::Covariance(data_edge[0].cov);
    graph.add(PriorFactor<Pose3> (0, priorMean, prior_noise));

    /* CREATING A NON-LINEAR-FACTOR-GRAPH */
    for(int i=0; i<data_edge.size(); i++){
        auto cov =  gtsam::noiseModel::Gaussian::Covariance(data_edge[i].cov);
        graph.add(BetweenFactor<Pose3>(data_edge[i].i, data_edge[i].j, Pose3(Rot3(data_edge[i].rot), data_edge[i].translation), cov));
    }
    // graph.print("Graph");
    //printGraph(graph);

    /* OPTIMIZATION SOLUTION */
    gtsam::GaussNewtonParams params;
    params.setVerbosity("TERMINATION");
    gtsam::GaussNewtonOptimizer optimizer(graph, node, params);
    gtsam::Values result = optimizer.optimize();
    //result.print();

    return result;
}

gtsam::Values solution_Incremental(gtsam::Values node, gtsam::NonlinearFactorGraph graph, vector<Pose> data_pose, vector<Edge> data_edge){
    /* INCREMENTAL SOLUTION VARIABLES */
    gtsam::ISAM2 isam;
    gtsam::Values optimal;

    /* ALGORITHM LOOP */
    for(auto pose : data_pose){
        
        if(pose.id == 0){
            gtsam::Pose3 priorMean(Rot3(data_pose[0].rot), data_pose[0].translation);
            auto prior_noise = gtsam::noiseModel::Gaussian::Covariance(data_edge[0].cov);
            graph.add(PriorFactor<Pose3> (0, priorMean, prior_noise));
            node.insert(pose.id, priorMean);
        }
        else{
            gtsam::Pose3 prev_pose = optimal.at<gtsam::Pose3>(pose.id - 1);
            node.insert(pose.id, prev_pose);
            for(auto edge : data_edge){
                if(pose.id == edge.j){
                    auto cov = gtsam::noiseModel::Gaussian::Covariance(edge.cov);
                    graph.add(BetweenFactor<Pose3>(edge.i, edge.j, Pose3(Rot3(edge.rot), edge.translation), cov));
                }
            }
        }
        //graph.print("Graph");
        //node.print("Node");
        
        isam.update(graph, node);
        optimal = isam.calculateEstimate();
        graph.resize(0);
        node.clear();
        //optimal.print("optimal: \n");
    }

    cout<<"INCREMENTAL COMPLETE"<<endl;
    return optimal;
}

void resultsToMatlab(gtsam::Values results, string filename){
    ofstream file(filename);
    for(int i=0; i < results.size(); i++){
        file << results.at<gtsam::Pose3>(i).x()<<" "<<results.at<gtsam::Pose3>(i).y()<<" " << results.at<gtsam::Pose3>(i).z() <<endl;
    }
    file.close();
}

void givenDataToMatlab(vector<Pose> data_pose, string filename){
    ofstream file(filename);
    for(int i=0; i<data_pose.size();i++){
        file << data_pose[i].x << " " <<data_pose[i].y <<" "<< data_pose[i].z<<endl;
    }
    file.close();
}


int main(){
    cout<<" 3D SLAM "<<endl;

    std::string file = "/home/aditya/ROB530/HW/parking-garage.g2o";
    vector<Pose> data_pose;
    vector<Edge> data_edge;

    tie(data_pose,data_edge) = loadg2o(file);

    //printPose(data_pose);
    //printEdge(data_edge);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values node;

    cout<< " DATA LOADED" << endl;

    //***** BATCHED FUNCTION CALL ******//
    gtsam::Values result_batched = solution_Batch(node, graph, data_pose, data_edge);

    //***** INCREMENTAL FUNCTION CALL ******//
    gtsam::Values optimized = solution_Incremental(node, graph, data_pose, data_edge);

    /* PRINT FUNCTIONS */
    //optimized.print("OPTIMIZED SOLUTION");

    /* ONLY WHEN RUNNING FINAL CODE */
    /*********************************/
    //resultsToMatlab(result_batched, "batched_solution_3D.txt");
    //givenDataToMatlab(data_pose, "unoptimized_pose_3D.txt");
    //resultsToMatlab(optimized, "incremental_solution_3D.txt");
    /*********************************/

    return 0;
    
}

