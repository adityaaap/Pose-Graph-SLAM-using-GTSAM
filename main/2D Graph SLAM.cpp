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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>


using namespace gtsam;
using namespace std;

struct Pose{
    double id;
    double x;
    double y;
    double theta;
    
    Pose(double id, double x,double y,double theta) : id(id), x(x), y(y), theta(theta) {}
};

struct Edge{
    int i;
    int j;
    double x;
    double y;
    double theta;
    Eigen::Matrix<double, 3, 3> cov;
    

    Edge(int i, int j, double x, double y, double theta, double arg[]): i(i), j(j), x(x), y(y), theta(theta){
        
        cov(0,0) = arg[0];
        cov(0,1) = arg[1];
        cov(1,0) = arg[1];
        cov(0,2) = arg[2];
        cov(2,0) = arg[2];
        cov(2,1) = arg[4];
        cov(1,2) = arg[4];
        cov(1,1) = arg[3];
        cov(2,2) = arg[5];

        /* COVERTING INFO TO COV */
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

    vector<Pose> data_pose;
    vector<Edge> data_edge;
    double i,j,k,l;
    double id1, id2, x, y, theta, q1, q2, q3, q4, q5, q6;

    std::string line;
    
    // while(infile.good()){
    //     infile >> line;
    //     std::cout<< line << std::endl;
    // }

    while(infile.good()){
        infile >> line;
        //std::cout<< line << std::endl;

        if(line == "VERTEX_SE2"){
            infile >> i >> j >> k >> l;
            data_pose.push_back(Pose(i,j,k,l));
        }
        else{
            if(line == "EDGE_SE2"){
                infile >> id1 >> id2 >> x >> y >> theta >> q1 >> q2 >> q3 >> q4 >> q5 >> q6; 
                double arr[] = {q1, q2, q3, q4, q5, q6};
                data_edge.push_back(Edge(id1, id2, x, y, theta, arr));
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
        std::cout<<"theta: "<<data_edge[i].theta<<std::endl;
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
        cout<<"theta "<<data_pose[i].theta<<endl;
        cout<<endl;
    }
}

void printNodes(gtsam::Values node){
    node.print("Nodes");
}

void printGraph(NonlinearFactorGraph graph){
    graph.print("Graph");
}

gtsam::Values solution_Batch(gtsam::Values node, gtsam::NonlinearFactorGraph graph, vector<Pose> data_pose, vector<Edge> data_edge){
    /* INSERTING POSE DATA INTO NODES(initial values) */
    for(int i=0; i<data_pose.size(); i++){
        node.insert(data_pose[i].id, Pose2(data_pose[i].x, data_pose[i].y, data_pose[i].theta));
    }
    //node.print("Nodes");
    //printNodes(node);

    /* ADDING PRIOR */
    gtsam::Pose2 priorMean(data_pose[0].x, data_pose[0].y, data_pose[0].theta);
    auto prior_noise = gtsam::noiseModel::Gaussian::Covariance(data_edge[0].cov);
    graph.add(PriorFactor<Pose2> (0, priorMean, prior_noise));

    /* CREATING A NON-LINEAR-FACTOR-GRAPH */
    for(int i=0; i<data_edge.size(); i++){
        auto cov =  gtsam::noiseModel::Gaussian::Covariance(data_edge[i].cov);
        graph.add(BetweenFactor<Pose2>(data_edge[i].i, data_edge[i].j, Pose2(data_edge[i].x, data_edge[i].y, data_edge[i].theta), cov));
    }
    // graph.print("Graph");
    //printGraph(graph);

    /* OPTIMIZING THE GRAPH */
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
            gtsam::Pose2 priorMean(pose.x, pose.y, pose.theta);
            auto prior_noise = gtsam::noiseModel::Gaussian::Covariance(data_edge[0].cov);
            graph.add(PriorFactor<Pose2> (0, priorMean, prior_noise));
            node.insert(pose.id, priorMean);
            
        }
        else{
            gtsam::Pose2 prev_pose = optimal.at<gtsam::Pose2>(pose.id - 1);
            node.insert(pose.id, prev_pose);
            for(auto edge : data_edge){
                if(pose.id == edge.j){
                    auto cov = gtsam::noiseModel::Gaussian::Covariance(edge.cov);
                    graph.add(BetweenFactor<Pose2>(edge.i, edge.j, Pose2(edge.x, edge.y, edge.theta), cov));
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

    cout<<"ALGO COMPLETE!!!!"<<endl;
    return optimal;
}

void resultsToMatlab(gtsam::Values results, string filename){
    ofstream file(filename);
    for(int i=0; i < results.size(); i++){
        file << results.at<gtsam::Pose2>(i).x()<<" "<<results.at<gtsam::Pose2>(i).y() <<endl;
    }
    file.close();
}

void givenDataToMatlab(vector<Pose> data_pose, string filename){
    ofstream file(filename);
    for(int i=0; i<data_pose.size();i++){
        file << data_pose[i].x << " " <<data_pose[i].y <<endl;
    }
    file.close();
}



int main(){

    std::string file = "/home/aditya/ROB530/HW/input_INTEL_g2o.g2o";
    vector<Pose> data_pose;
    vector<Edge> data_edge;

    tie(data_pose,data_edge) = loadg2o(file);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values node;

    //***** BATCHED FUNCTION CALL ******//
    gtsam::Values result_batched = solution_Batch(node, graph, data_pose, data_edge);
    //resultsToMatlab(result_batched, "batched_solution_2D.txt");

    //***** INCREMENTAL FUNCTION CALL ******//
    gtsam::Values optimized = solution_Incremental(node, graph, data_pose, data_edge);
    //resultsToMatlab(optimized, "incremental_solution_2D.txt");
    //result_batched.print("results");


    /* ONLY WHEN RUNNING FINAL CODE */
    /*********************************/
    // resultsToMatlab(optimized, "incremental_solution_2D.txt");
    // resultsToMatlab(result_batched, "batched_solution_2D.txt");
    // givenDataToMatlab(data_pose, "unoptimzed_pose_2D.txt");
    /*********************************/

    return 0;
}