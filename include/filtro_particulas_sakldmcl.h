#ifndef FILTRO_PARTICULAS_SAKLDMCL_H
#define FILTRO_PARTICULAS_SAKLDMCL_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/Marker.h>

#include "map_server/image_loader.h"
#include "nav_msgs/GetMap.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

#include "filtro_particulas_sakldmcl/grid_pose_energy.h"


#define sign(a) (((a) < 0) ? -1 : (((a) > 0) ? 1 : 0))

using namespace std;

class Filtro_Particulas_Sakldmcl
{
	public:
		Filtro_Particulas_Sakldmcl(ros::NodeHandle n);
		virtual ~Filtro_Particulas_Sakldmcl();

	public:
		void readLandmarks();
		void createParticles();

		void fakeLaser();
		void fakeAllLaserBeam();
		double findObstacle(double x, double y);
		void moveParticles();
		void weightParticles();
		void resample();
		double measurementProb(int particleMP, int laserMP);

		double gaussian(double mu, double sigma, double x);
		double gaussian(double mu, double sigma);

		void odomCallback (const nav_msgs::OdometryConstPtr& msg);
		void laserCallback (const sensor_msgs::LaserScanConstPtr& scan);
		void occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy);
		void free_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& free_coordxy);
		void mapCallback (const nav_msgs::MapMetaDataConstPtr& msg);

		void pubInicialPose();
		void cloud();
		void visualizationMarker();

		void createGrids();
		void ordenaGrid();
		void merge_sort (filtro_particulas_sakldmcl::grid_pose_energy vector[], const int low, const int high);
		void merge (filtro_particulas_sakldmcl::grid_pose_energy vector[], const int low, const int mid, const int high);
		void calculoSER();
		bool buscaEnergiaSER();

		void calculoNumKBins();
		bool buscaPosexyBins();
		bool ordenaVetorPosexyBins();
		void merge_sort (int vector[], const int low, const int high);
		void merge (int vector[], const int low, const int mid, const int high);
		void calculoSampleSize(int k);
		void escreveTxt();
		void kidnapping();

		void spin();

	private:
		ros::NodeHandle n_;

		nav_msgs::Odometry odometry_;

		ros::Subscriber odom_sub_;
		ros::Subscriber scan_sub_;
		ros::Subscriber occ_coordxy_sub_;
		ros::Subscriber free_coordxy_sub_;
		ros::Subscriber map_meta_data_sub_;

		ros::Publisher particle_cloud_pub_;
		ros::Publisher initial_pose_pub_;
		ros::Publisher particle_curr_pose_pub_;
		ros::Publisher marker_converted_pose_pub_;


		geometry_msgs::Pose2D single_pose_;
		geometry_msgs::Pose2D particle_pose_[10000];
		geometry_msgs::Pose2D particle_resample_[10000];
		geometry_msgs::PoseWithCovarianceStamped initial_pose_;
		geometry_msgs::Pose2D initial_pose2_;
		geometry_msgs::PoseArray pose_curr_msg;

		visualization_msgs::Marker points_, line_strip_, line_list_;

		filtro_particulas_sakldmcl::grid_pose_energy grid_pose_energy_[100000];
		//filtro_particulas_sakldmcl::grid_pose_energy grid_sorted_[100000];

		//variáveis para criar aquivo de media e desvio padrao a partir do bag
		ros::Time time_bag_inicio;
		ros::Time time_bag_curr;
		ros::Duration time_bag_dif;
		int pos_amostra_;
		ofstream myfile_;
		int num_bag_amostras_;
		int qtdd_amostras_;
		double seq_laser_;
		double seq_laser_ant_;
		int r_bag_;

		double weight_all_laserbeam_;

		double map_meta_data_;
		double res_;
		double map_position_x_;
		double map_position_y_;

		//Variáveis para medição do tempo para as partículas convergirem
		ros::Time time_initial_sec_;
		ros::Time time_converged_sec_;
		bool save_initial_time_;
		bool time_converged_ok_;
		ros::Duration time_to_converge_sec_;

		int num_part_;
		int qtdd_laser_;
		int qtdd_orient_;
		double passo_base;
		double range_max_fakelaser; //[m]
		double error_particles_;
		double dist_threshold_;
		bool prim_converg_;

		//variáveis para SAMCL
		int num_part_local_;
		int num_part_global_;
		double weight_threshold_;
		double alpha_sample_set_;

		int num_energy_;
		int size_grid_energy_;
		int sorted_indice_;
		int grid_indice_sorted_[100000];
		double grid_sorted_[100000];
		double laser_data_energy_;
		int indice_busca_SER_[100000];
		int num_particulas_SER_;
		bool calculo_SER_loop_;

		double ser_threshold_;

		double fator_part_threshold_;

		//variáveis para KLD
		int max_part_;
		int min_part_;
		int num_min_part_;
		double kld_err_;
		double kld_z_;
		int bins_;
		int k_bins_;
		int vetor_pose_xy_bins_[10000];
		int pose_xy_bins_;


		double reduz_gauss_;
		double arctan_;
		double hipot_;
		double freq_;
		int landmarks_[4000][2];
		int landmarks_xy_[4000];
		int free_xy_[40000];
		int l_;
		int f_;
		int num_free_;
		int min_x_;
		int min_y_;
		int max_x_;
		int max_y_;
		bool obstacle_finded_;
		int obstacle_;
		int achou;
		int loop;
		int cont;
		double total;
		double probt;
		double probtg;
		double passo;
		double sum;
		int index_max_w_;

		int rand_xy;
		double pose_x;
		double pose_y;
		double twist_x_;

		double x;
		double y;
		int xi;
		int yi;
		int i;
		int num_laser;

		geometry_msgs::Pose2D delta_pose_;
		geometry_msgs::Pose2D pose_anterior_;
		geometry_msgs::Pose2D pose;
		geometry_msgs::Pose2D fake_laser_pose_[1000];

		double laser_data_[1000];
		double ang_min_;
		double max_laser_range_;
		double pose_x_;
		double pose_y_;
		double pose_theta_;
		double fake_laser_data_[10000][3];
		double gaussian_;
		double move_noise_;
		double turn_noise_;
		double laser_noise_;
		double laser_data_noise_;
		int size_occ_coordxy_;
		double weight_part_laser_[10000][3];
		double weight_part_[10000];
		double pool_[10000];
		double weight_ord_[10000];
		double max_w_;
		double media_pesos_;

		bool free_ok_;
		bool occ_ok_;
		bool odom_ok_;
		bool laser_ok_;
		bool zerar_deltas_;
		bool create_particle_ok_;
		bool grids_ok_;
		bool calculo_SER_ok_;
		bool busca_energia_SER_ok_;
		bool abriu_txt_ok_;
		bool zerar_time_ok_;
		bool reduziu_num_part_;
		bool kidnapping_ok_;
};

#endif
