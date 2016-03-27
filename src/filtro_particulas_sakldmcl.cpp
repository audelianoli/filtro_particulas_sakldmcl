#include "filtro_particulas_sakldmcl.h"

Filtro_Particulas_Sakldmcl::Filtro_Particulas_Sakldmcl(ros::NodeHandle n) : myfile_("/home/au/catkin_ws/20_Movimento_dados_SA-KLD_full.csv",ofstream::app)
{
	n_ = n;

	occ_coordxy_sub_ = n.subscribe("occ_coordxy", 4000, &Filtro_Particulas_Sakldmcl::occ_coordxyCallback, this);
	free_coordxy_sub_ = n.subscribe("free_coordxy", 4000, &Filtro_Particulas_Sakldmcl::free_coordxyCallback, this);
	scan_sub_ = n.subscribe("scan", 10, &Filtro_Particulas_Sakldmcl::laserCallback, this);
	odom_sub_ = n.subscribe("odom", 10, &Filtro_Particulas_Sakldmcl::odomCallback, this);
	map_meta_data_sub_ = n.subscribe("map_metadata", 10, &Filtro_Particulas_Sakldmcl::mapCallback, this);

	initial_pose_pub_ = n.advertise<geometry_msgs::Pose2D>("filterparticlepose", 1, true);
	particle_cloud_pub_ = n.advertise<geometry_msgs::PoseArray>("particlecloudAU", 2, true);
	particle_curr_pose_pub_ = n.advertise<geometry_msgs::PoseArray>("particle_curr_pose", 2, true);
	marker_converted_pose_pub_ = n.advertise<visualization_msgs::Marker>("visualization_mcl_pose", 10, true);

//--------------------------------------------------------------------------------//
	freq_ = 30.0;

	desabilita_kld_ = false;
	desabilita_samcl_ = false;
	desabilita_vary_sample_ = false;
	desabilita_kid_ = false;

	pos_amostra_ = 0;
	num_bag_amostras_ = 0;
	qtdd_amostras_ = 20;
	seq_laser_ant_ = 1000;
	r_bag_ = 3;
	bins_ = 2;
	iteration_ = 0;

	num_part_local_ = 0;

	max_part_ = 500;
	min_part_ = 500;
 	qtdd_laser_ = 30; //quantidade de pontos do laser que serão lidos
	qtdd_orient_ = 18; //quantidade de giros no mesmo pose. Atentar-se ao numero de free_xy. qtdd_orient*free_xy < 100mil
	range_max_fakelaser = 10; //5.6; //[m]

	//Ruídos do laser, movimento linear e movimento angular
	//noise_level = (desvio_padrao / max_range) * 100% || max_range * erro_desejado
	laser_data_noise_ = 0.28; //(0.28 / 5.6) ~ 5%
	move_noise_ = 0.020; //(0.016 / 0.2) ~ 8%
	turn_noise_ = 0.015; //(0.012 / 0.15) ~ 8%

	//parâmetro para convergir

	error_particles_ = 0.14; //0.45 ~ dist de 0.3m da particula (na media) ; 0.28 ~ 0.2m; 0.14 ~ 0.1m

	//parâmetros para SAMCL

	//ser_threshold_ = 0.024; //diferença entre a energia virtual e a energia real. Quanto maior, mais células serão utilizadas quando forem criadas as partículas
	ser_threshold_ = 0.020;
	//weight_threshold_ = 0.0030; //--> 5000 part //0.0015 -> 0.10 //Quanto maior o num_part_ maior deverá ser o weight_threshold
	weight_threshold_ = 0.05;
	alpha_sample_set_ = 0.6; //60% local e 40% global
	fator_part_threshold_ = 10000;//3 //10000 -> desabilita

	num_min_part_ = 10000;

	//parâmetros para KLD
	kld_err_ = 0.010; //quanto maior o erro, menor o número de partículas por k_bins
	kld_z_ = 2.33; // p-value = P(Z>2.33) = 1 - P(Z>2.33) = 1-0.9901 = 0.0099 //upper 1 − δ quantile of the standard normal distribution -> usar tabela de quantil

	sakld_err_ = 0.015;

//--------------------------------------------------------------------------------//

	reduz_gauss_ = 1.0; //aumenta 10% do erro gaussiano para diminuir as divergencias
	arctan_ = 0.0;
	hipot_ = 0.0;
	num_laser = 0;
	ang_min_ = 0;
	reduziu_num_part_ = false;
	l_ = 0;
	f_ = 0;
	min_x_ = 10000;
	min_y_ = 10000;
	max_x_ = -10000;
	max_y_ = -10000;
	num_energy_ = 0;
	size_grid_energy_ = 0;
	sorted_indice_ = 0;

	num_particulas_SER_ = 0;
	calculo_SER_loop_ = false;

	single_pose_.x = 0;
	single_pose_.y = 0;
	single_pose_.theta = 0;
	num_free_ = 0;
	pose_x_ = 0;
	pose_y_ = 0;
	pose_theta_ = 0;
	gaussian_ = 0;

	twist_x_ = 0.0;

	delta_pose_.x = 0;
	delta_pose_.y = 0;
	delta_pose_.theta = 0;
	pose_anterior_.x = 0;
	pose_anterior_.y = 0;
	pose_anterior_.theta = 0;

	pose.x = 0;
	pose.y = 0;
	pose.theta = 0;

	obstacle_finded_ = false;

	size_occ_coordxy_ = 0;
	obstacle_ = 0;
	achou = 0;
	loop = 0;
	cont = 0;
	total = 0;
	probt = 0;
	passo = 0;
	sum = 1000;
	index_max_w_ = 0;

	rand_xy = 0;
	pose_x = 0;
	pose_y = 0;

	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;

	time_conv_ = 0;

	occ_ok_ = false;
	odom_ok_ = false;
	laser_ok_ = false;
	free_ok_ = false;
	zerar_deltas_ = false;
	create_particle_ok_ = true;
	grids_ok_ = false;
	calculo_SER_ok_ = false;
	busca_energia_SER_ok_ = false;
	save_initial_time_ = false;
	time_converged_ok_ = false;
	abriu_txt_ok_ = false;
	zerar_time_ok_ = false;
	kidnapping_ok_ = false;
	out_ = false;
	calc_new_num_particles_ = true;
}

Filtro_Particulas_Sakldmcl::~Filtro_Particulas_Sakldmcl()
{
	occ_coordxy_sub_.shutdown();
	free_coordxy_sub_.shutdown();
	scan_sub_.shutdown();
	odom_sub_.shutdown();
	map_meta_data_sub_.shutdown();
	initial_pose_pub_.shutdown();
	particle_cloud_pub_.shutdown();
	particle_curr_pose_pub_.shutdown();
	marker_converted_pose_pub_.shutdown();
}

void Filtro_Particulas_Sakldmcl::mapCallback(const nav_msgs::MapMetaDataConstPtr& msg)
{
	map_meta_data_ = msg->resolution;
	res_ = map_meta_data_;
	passo_base = res_;
	cout<<"map resolution: "<<res_<<endl;

	map_position_x_ = msg->origin.position.x;
	map_position_y_ = msg->origin.position.y;
	cout<<"map_position_x_: "<<map_position_x_<<" | map_position_y_: "<<map_position_y_<<endl;

}

void Filtro_Particulas_Sakldmcl::occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy)
{
	//Carregar os valores de xy dos landmarks.

	l_ = 0;

	for(std::vector<int>::const_iterator it = occ_coordxy->data.begin() ; it != occ_coordxy->data.end(); ++it){

		landmarks_xy_[l_] = *it;
		l_++;
	}
	occ_ok_ = true;
	//cout<<"sizeof: "<<l_<<endl;

	return;
}

void Filtro_Particulas_Sakldmcl::free_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& free_coordxy)
{
	f_ = 0;

	for(std::vector<int>::const_iterator it = free_coordxy->data.begin() ; it != free_coordxy->data.end(); ++it){

		free_xy_[f_] = *it;
		f_++;
	}
	num_free_ = f_;
	//cout<<"num_free_coordxy: "<<num_free_<<endl;
	free_ok_ = true;

	return;
}

void Filtro_Particulas_Sakldmcl::laserCallback (const sensor_msgs::LaserScanConstPtr& scan)
{
//-------sempre que o bag der um novo loop--------------------------------------------------//
	seq_laser_ = scan->header.seq;
	if(seq_laser_ > seq_laser_ant_)
	{
		seq_laser_ant_ = seq_laser_;
	}
	else if(num_bag_amostras_ >= 0)
	{
		cout<<"time_bag_final: "<<time_bag_dif.toSec()<<endl;
		seq_laser_ant_ = 0;
		time_bag_inicio = ros::Time::now();
		zerar_time_ok_ = true;
		calculo_SER_ok_ = false;
		create_particle_ok_ = true;
		zerar_deltas_ = false;
		time_converged_ok_ = false;
		pos_amostra_ = 0;
		if(desabilita_vary_sample_ == false)
		{
			calc_new_num_particles_ = true;
		}else num_part_ = max_part_;
		iteration_ = 0;
	}
//---------------------------------------------------------//

	if(pos_amostra_ == 0) pos_amostra_ = 1;

	// 1,57 / 0,006 ~ 260
	// 3,14 / 0,01 = 360
	ang_min_ = scan -> angle_min;
	max_laser_range_ = scan->range_max;

	//int it = (scan->ranges.size() - 1) / (qtdd_laser_ - 1); //259 / (qtdd_laser_ - 1)
	int it = (scan->ranges.size()) / (qtdd_laser_); // 360 / (qtdd_laser_)

	for (int laser_num = 0 ; laser_num < qtdd_laser_ ; laser_num++)
	{
		if(scan -> ranges[laser_num * it] >= scan->range_min && scan -> ranges[laser_num * it] <= scan->range_max)
		{
			laser_data_[laser_num] = scan -> ranges[laser_num * it];
		}
		else
		{
			laser_data_[laser_num] = -1;
			//cout<<"DEU NAN OU INF!!!!  "<<laser_num<<" = "<<laser_data_[laser_num]<<endl;
		}
	}
	laser_ok_ = true;
}

void Filtro_Particulas_Sakldmcl::odomCallback (const nav_msgs::OdometryConstPtr& msg)
{
	pose_x_ = msg->pose.pose.position.x;
	pose_y_ = msg->pose.pose.position.y;
	pose_theta_ = tf::getYaw(msg->pose.pose.orientation);

	twist_x_ = msg->twist.twist.linear.x;

	if((msg->twist.twist.linear.x != 0 || msg->twist.twist.angular.z != 0 )&& save_initial_time_ == false)
	{
		time_initial_sec_ = ros::Time::now();
		save_initial_time_ = true;
	}
	odom_ok_ = true;
}

void Filtro_Particulas_Sakldmcl::calculoNumKBinsForCreateParticles()
{
	//Método para carregar o vetor_pose_xy_bins_[] com as células intervaladas a cada bin.
	//zerar o vetor
	for(int j = 0 ; j < 10000 ; j++)
	{
		vetor_pose_xy_bins_[j] = 0;
	}

	k_bins_ = 0;

	int pose_x_pixel = 0;
	int pose_y_pixel = 0;

	int celula_pose_x = 0;
	int celula_pose_y = 0;

	for(int i = 0 ; i < num_particulas_SER_ ; i++)
	{
		//pose em pixels
		pose_x_pixel = (grid_pose_energy_[grid_indice_sorted_[indice_busca_SER_[i]]].xy)/10000;
		pose_y_pixel = (grid_pose_energy_[grid_indice_sorted_[indice_busca_SER_[i]]].xy)%10000;

		//relacionar os poses com os bins
		celula_pose_x = pose_x_pixel / bins_;
		celula_pose_y = pose_y_pixel / bins_;

		pose_xy_bins_ = (10000 * celula_pose_x) + celula_pose_y;

		//busca o valor de pose_xy_bins_
		buscaPosexyBins();
		//se existir, não faz nada. Se não, adicionar no início do vetor vetor_pose_xy_bins_,
		//incrementar o número de bins ocupados pelas partículas
		//e ordenar este vetor
		if(buscaPosexyBins() == false)
		{
			vetor_pose_xy_bins_[0] = pose_xy_bins_;
			k_bins_++;
			ordenaVetorPosexyBins();
		}
	}
	calculoSampleSizeForCreateParticles(k_bins_);
}
void Filtro_Particulas_Sakldmcl::calculoSampleSizeForCreateParticles(int k)
{
	double a, b, c, x;
	int n;

	if (k <= 1)
	{
		//ROS_INFO("k: %d !!! Não devia...", k);
	}
	//Cálculo do número de partículas utilizando a equação de KL-Distance
	else
	{
		a = 1;
		b = 2 / (9 * ((double) k - 1));
		c = sqrt(2 / (9 * ((double) k - 1))) * kld_z_;
		x = a - b + c;

		n = (int) ceil((k - 1) / (2 * sakld_err_) * x * x * x);

		if (n < min_part_)
		{
			num_part_ = min_part_;
		}
		else if (n > max_part_)
		{
			num_part_ = max_part_;
		}
		else
		{
			num_part_ = n;
		}
		cout<<"Para CREATE PARTICLES: Num_part_KLD: "<<n<<" | num_SER: "<<num_particulas_SER_<<" | k_bins_: "<<k_bins_<<" | bins: "<<bins_<<" | sakld_err: "<<sakld_err_<<" | kld_z: "<<kld_z_<<endl;
		//printf("Number_Particle: %d | k_bins: %d | bins: %d | max_weight: %f | num_min_part: %d \n", num_part_, k_bins_, bins_,max_w_, num_min_part_);

		myfile_<<"Num_part_KLD: "<<n<<","<<"num_SER: "<<num_particulas_SER_<<","<<"k_bins_: "<<k_bins_<<","<<"bins: "<<bins_<<","<<"sakld_err: "<<sakld_err_<<","<<"kld_z: "<<kld_z_<<",";

	}
	calc_new_num_particles_ = false;
}


void Filtro_Particulas_Sakldmcl::createParticles()
{
	//Criando partículas randômicas.

	//mudando a semente do random
	if(create_particle_ok_ == true && calc_new_num_particles_ == false)
	{
		cout<<"Particles Created!!!!!"<<endl;
		cout<<"Local_Particles: "<<num_part_local_<<" | Global_Particles: "<<num_part_global_<<" | Total_Particles: "<<num_part_<<endl;
		srand(time(NULL));

		rand_xy = 0;
		pose_x = 0;
		pose_y = 0;

		for (int i = num_part_local_; i < num_part_; i++)
		{
			if(desabilita_samcl_ == true)
			{
				rand_xy = rand() % num_free_; //random de 0 a num_free_

				pose_x = (free_xy_[rand_xy])/10000; //separa x de y
				pose_y = (free_xy_[rand_xy])%10000;

				single_pose_.x = pose_x * res_; //1 pixel -> 0.05m
				single_pose_.y = pose_y * res_;
				single_pose_.theta = (rand() % 360 + 0) - 180; //em graus
				single_pose_.theta = single_pose_.theta * M_PI / 180; //em radianos

				particle_pose_[i] = single_pose_;
			}else
			{
				rand_xy = rand() % num_particulas_SER_; //random de 0 a num_particulas_SER_

				pose_x = (grid_pose_energy_[grid_indice_sorted_[indice_busca_SER_[rand_xy]]].xy)/10000; //separa x de y
				pose_y = (grid_pose_energy_[grid_indice_sorted_[indice_busca_SER_[rand_xy]]].xy)%10000;

				single_pose_.x = pose_x * res_; //1 pixel -> 0.05m
				single_pose_.y = pose_y * res_;
				single_pose_.theta = (rand() % 360 + 0) - 180; //em graus
				single_pose_.theta = single_pose_.theta * M_PI / 180; //em radianos

				particle_pose_[i] = single_pose_;
			}
		}
		cloud();
		prim_converg_ = false;
		create_particle_ok_ = false;
		sum = 1000;
		kidnapping_ok_ = false;
		//busca_energia_SER_ok_ = false;
	}
}

double Filtro_Particulas_Sakldmcl::gaussian(double mu, double sigma, double x)
{
	double a;
	double b;

	a = (mu - x) * (mu - x);
	b = sigma * sigma;
	gaussian_ = (exp(- a / b / 2.0)) / (sqrt(2.0 * M_PI) * sigma);

	return gaussian_;

}

double Filtro_Particulas_Sakldmcl::gaussian(double mu, double sigma)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::normal_distribution<double> distribuition(mu,sigma);

	double number = distribuition(generator);

	return number;
}

void Filtro_Particulas_Sakldmcl::fakeLaser()
{
	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;
	num_laser = 0;
	achou = 0;
	total = 0;
	max_w_before_norm_ = -1;

	for (int i = 0; i < num_part_; i++)
	{
		//probt = 1.0;
		probt = 0.0;

		double it = M_PI / (qtdd_laser_);

		for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
		{
			fake_laser_pose_[num_laser].theta = ((ang_min_) + (num_laser * it)) + particle_pose_[i].theta;
			if(fake_laser_pose_[num_laser].theta > M_PI)
				fake_laser_pose_[num_laser].theta -= 2.0 * M_PI;
			if(fake_laser_pose_[num_laser].theta <= - M_PI)
				fake_laser_pose_[num_laser].theta += 2.0 * M_PI;

			fake_laser_pose_[num_laser].x = particle_pose_[i].x;
			fake_laser_pose_[num_laser].y = particle_pose_[i].y;
		}

		for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
		{
			passo = 0;
			int iteracao = 7.0 / passo_base; //range_max_fakelaser / passo_base; // 5.6 / 0.06 = 112

			for(int p = 1; p <= iteracao; p++)
			{
				//varredura do fake_laser
				passo = (passo_base * p); //+ gaussian(0,laser_data_noise_);
				//cout<<"passo: "<<passo<<endl;
				x = fake_laser_pose_[num_laser].x + (cos(fake_laser_pose_[num_laser].theta) * passo);
				y = fake_laser_pose_[num_laser].y + (sin(fake_laser_pose_[num_laser].theta) * passo);

				//arredondando os valores de x e y
				xi = x / res_;
				yi = y / res_;

				findObstacle(xi, yi);
				if (obstacle_finded_ == true)
				{
					fake_laser_data_[i][num_laser] = obstacle_;
					p = iteracao;
				}else fake_laser_data_[i][num_laser] = 0;

			}
			weight_part_laser_[i][num_laser] = passo; //DISTANCIA FAKE DE CADA FEIXE DO LASER VIRTUAL!!!

			if(laser_data_[num_laser] > 0)
				measurementProb(i,num_laser);
		}
		probt = 1/probt;

		weight_part_[i] = probt;

		total += weight_part_[i];

		if(weight_part_[i] > max_w_before_norm_)
		{
			max_w_before_norm_ = weight_part_[i];
		}
	}
}

double Filtro_Particulas_Sakldmcl::findObstacle(double x, double y)
{
	int esq, meio, dir;
	esq = 0;
	dir = l_ - 1;

	obstacle_ = (10000 * x) + y;

	while (esq <= dir){
		meio = (esq + dir) / 2;
		if ( landmarks_xy_[meio] == obstacle_)
		{
			obstacle_finded_ = true;
			achou++;
			return obstacle_;
		}
		if ( landmarks_xy_[meio] < obstacle_) esq = meio + 1;
		else dir = meio - 1;
	}
	obstacle_finded_ = false;
	return -1;
}

double Filtro_Particulas_Sakldmcl::measurementProb(int particleMP, int laserMP)
{
	probt +=  fabs(weight_part_laser_[particleMP][laserMP] - (laser_data_[laserMP])); //+ gaussian(0,laser_data_noise_)));

	return probt;
}

void Filtro_Particulas_Sakldmcl::resample()
{
	double soma = 0;
	max_w_ = -1;
	media_pesos_ = 0;
	for(int n = 0 ; n < num_part_ ; n++)
	{
		//Normaliza os pesos
		weight_part_[n] = weight_part_[n] / total;

		if(weight_part_[n] > max_w_)
		{
			max_w_ = weight_part_[n];
			index_max_w_ = n;
		}
	}

	int index_b = 0;
	int indexi = 0;
	srand(time(NULL));
	index_b = rand() % 101;
	indexi = index_b * num_part_ / 100;
	int beta_b = 0;
	double beta = 0;

	for (int p = 0; p < num_part_; p++)
	{
		srand(time(NULL));
		beta_b = rand() % 101;
		beta += (beta_b) * 2.0 * max_w_ / 100.0;
		while(beta > weight_part_[indexi])
		{
			beta -= weight_part_[indexi];
			indexi = (indexi + 1) % num_part_;
		}
		particle_resample_[p] = particle_pose_[indexi];
	}
	for(int n = 0 ; n < num_part_ ; n++)
	{
		particle_pose_[n] = particle_resample_[n];
	}
}

void Filtro_Particulas_Sakldmcl::moveParticles()
{
	delta_pose_.x = pose_x_ - pose_anterior_.x;
	delta_pose_.y = pose_y_ - pose_anterior_.y;
	delta_pose_.theta = pose_theta_ - pose_anterior_.theta;
	hipot_ = sqrt((delta_pose_.x * delta_pose_.x) + (delta_pose_.y * delta_pose_.y));
	arctan_ = atan(delta_pose_.y / delta_pose_.x);

	pose_anterior_.x = pose_x_;
	pose_anterior_.y = pose_y_;
	pose_anterior_.theta = pose_theta_;

	int p = 0;
	if(delta_pose_.x != 0 || delta_pose_.y != 0 || delta_pose_.theta != 0)
	{
		for(int p = 0; p < num_part_; p++)
		{
			particle_pose_[p].x += sign(twist_x_) * hipot_ * cos(particle_pose_[p].theta) + (gaussian(0.0, move_noise_));
			particle_pose_[p].y += sign(twist_x_) * hipot_ * sin(particle_pose_[p].theta) + (gaussian(0.0, move_noise_));

			particle_pose_[p].theta += delta_pose_.theta + (gaussian(0.0, turn_noise_));

			if(particle_pose_[p].theta > M_PI)
				particle_pose_[p].theta -= 2.0 * M_PI;
			if(particle_pose_[p].theta <= - M_PI)
				particle_pose_[p].theta += 2.0 * M_PI;
		}
		cloud();
		fakeLaser();
		resample();
		pubInicialPose();
		if(desabilita_kld_ == false)
		{
			if(desabilita_samcl_ == true)
			{
				calculoNumKBins();
				calculoSampleSize(k_bins_);
			}else //if( (max_part_ > 2500) || ((sum < 2*error_particles_) && max_part_ <= 2500) )
			{
				calculoNumKBins();
				calculoSampleSize(k_bins_);
			}
		}
		//cout<<"Num_part: "<<num_part_<<endl;
	}
}

void Filtro_Particulas_Sakldmcl::pubInicialPose()
{
	double xmedia = 0.0;
	double ymedia = 0.0;
	double thetamedia = 0.0;
	double thetapos = 0.0;
	double thetaneg = 0.0;
	double pos = 0.0;
	double neg = 0.0;
	double dx, dy, err;
	sum = 0.0;

	for (int i = 0; i < num_part_ ; i++)
	{
		xmedia += particle_pose_[i].x;
		ymedia += particle_pose_[i].y;
		thetamedia = particle_pose_[i].theta;

		if(thetamedia < 0.0)
		{
			neg++;
			thetaneg += particle_pose_[i].theta;
		}else
		{
			pos++;
			thetapos += particle_pose_[i].theta;
		}
	}

	xmedia = xmedia / (double)num_part_;
	ymedia = ymedia / (double)num_part_;
	if (neg > pos)
	{
		thetaneg = thetaneg / neg;
		thetamedia = thetaneg;
	}else{
		thetapos = thetapos / pos;
		thetamedia = thetapos;
	}

	if(thetamedia > M_PI)
		thetamedia -= 2.0 * M_PI;
	if(thetamedia <= - M_PI)
		thetamedia += 2.0 * M_PI;

	for (int i = 0; i < num_part_ ; i++)
	{
		dx = particle_pose_[i].x - xmedia;
		dy = particle_pose_[i].y - ymedia;
		err = sqrt( (dx * dx) + (dy * dy) );

		sum += err;

	}
	sum = sum / num_part_;

	initial_pose_out_.x = xmedia;
	initial_pose_out_.y = ymedia;
	initial_pose_out_.theta = thetamedia;

	if((1.35 - pose_out_msg.poses[0].position.x < 0 || 1.35 - pose_out_msg.poses[0].position.x > 7.5 || 1.75 - pose_out_msg.poses[0].position.y < 0 || 1.75 - pose_out_msg.poses[0].position.y > 4))
	{
		//cout<<"x: "<<1.35 - pose_out_msg.poses[0].position.x<<" | y: "<<1.75 - pose_out_msg.poses[0].position.y<<endl;
		out_ = true;
	}

	if(sum < error_particles_)
	{
		//Para publicar o pose médio.
		initial_pose2_.x = xmedia;
		initial_pose2_.y = ymedia;
		initial_pose2_.theta = thetamedia;

		initial_pose_pub_.publish(initial_pose2_);

		if(time_converged_ok_ == false)
		{
			time_converged_ok_ = true;
			time_converged_sec_ = ros::Time::now();
			time_to_converge_sec_ = time_converged_sec_ - time_bag_inicio;

			time_conv_ = time_to_converge_sec_.toSec();

			//num_bag_amostras_++;
			//myfile_<<t<<endl;

			myfile_<<"time: "<<time_conv_<<",";

			//x = 1.30 - pose_curr_msg.poses[0].position.x;
			//y = 1.70 - pose_curr_msg.poses[0].position.y;
			//myfile_<<x<<","<<y<<",";

			cout<<"time_to_Converge: "<<time_conv_<<endl;
		}

		//visualizationMarker();

		prim_converg_ = true;
	}
}

void Filtro_Particulas_Sakldmcl::cloud()
{
	//publicar a nuvem de partículas
	geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
	cloud_msg.poses.resize(num_part_);
	for(int i = 0;i<num_part_;i++)
	{
		tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(particle_pose_[i].theta),
				tf::Vector3(particle_pose_[i].x + map_position_x_, particle_pose_[i].y + map_position_y_, 0)),cloud_msg.poses[i]);
	}
	particle_cloud_pub_.publish(cloud_msg);

	//publicar o pose_atual
	pose_curr_msg.header.stamp = ros::Time::now();
	pose_curr_msg.header.frame_id = "map";
	pose_curr_msg.poses.resize(1);
	tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(initial_pose2_.theta),
				tf::Vector3(initial_pose2_.x + map_position_x_, initial_pose2_.y + map_position_y_, 0)),pose_curr_msg.poses[0]);

	particle_curr_pose_pub_.publish(pose_curr_msg);

	pose_out_msg.header.stamp = ros::Time::now();
	pose_out_msg.header.frame_id = "map";
	pose_out_msg.poses.resize(1);
	tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(initial_pose_out_.theta),
				tf::Vector3(initial_pose_out_.x + map_position_x_, initial_pose_out_.y + map_position_y_, 0)),pose_out_msg.poses[0]);

}

void Filtro_Particulas_Sakldmcl::visualizationMarker()
{
	points_.header.frame_id = line_strip_.header.frame_id = line_list_.header.frame_id = "map";
	points_.header.stamp = line_strip_.header.stamp = line_list_.header.stamp = ros::Time::now();
	points_.ns = line_strip_.ns = line_list_.ns = "points_and_lines";
	points_.action = line_strip_.action = line_list_.action = visualization_msgs::Marker::ADD;

	points_.id = 0;
	line_strip_.id = 1;
	line_list_.id = 2;

	points_.type = visualization_msgs::Marker::POINTS;
	line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
	line_list_.type = visualization_msgs::Marker::LINE_LIST;

	// POINTS markers use x and y scale for width/height respectively
	points_.scale.x = points_.scale.y = 0.03;
	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip_.scale.x = 0.1;
	line_list_.scale.x = 0.1;

	// Points are green
	points_.color.g = 1.0f;
	points_.color.a = 1.0;

	// Line strip is blue
	line_strip_.color.b = 1.0;
	line_strip_.color.a = 1.0;

	// Line list is red
	line_list_.color.r = 1.0;
	line_list_.color.a = 1.0;

	geometry_msgs::Point p;
	p.x = initial_pose2_.x + map_position_x_;
	p.y = initial_pose2_.y + map_position_y_;

	points_.points.push_back(p);

	marker_converted_pose_pub_.publish(points_);

}

void Filtro_Particulas_Sakldmcl::createGrids()
{
	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;
	//num_laser = 0;
	achou = 0;
	total = 0;

	double ang_it = 2 * M_PI / qtdd_orient_;
	int grid_pose_x = 0;
	int grid_pose_y = 0;
	double it = M_PI / (qtdd_laser_);

	for (int i = 0; i < num_free_; i++)
	{
		//carrega o xy no grid[] com indice fator=6
		grid_pose_energy_[i*qtdd_orient_].xy = free_xy_[i];
		grid_pose_x = (free_xy_[i] / 10000) * res_;
		grid_pose_y = (free_xy_[i] % 10000) * res_;

		for(int ang_inc = 0 ; ang_inc < qtdd_orient_ ; ang_inc++)
		{
			//carrega os outros 6 grid[].xy com o mesmo pose_xy
			grid_pose_energy_[(i*qtdd_orient_) + ang_inc + 1].xy = free_xy_[i];
			grid_pose_energy_[(i*qtdd_orient_) + ang_inc].energy = 0.0;
			grid_pose_energy_[(i*qtdd_orient_) + ang_inc].sum = 0.0;

			for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
			{
				//carrega cada grid.theta com um giro diferente para cada um dos 6 pose.xy
				grid_pose_energy_[(i*qtdd_orient_) + ang_inc].theta = (ang_it * ang_inc);

				//carrega o fake_laser[].theta com cada theta-ésimo ponto do fake_laser para o mesmo grid.theta
				fake_laser_pose_[num_laser].theta = ((ang_min_) + (num_laser * it)) + grid_pose_energy_[(i*qtdd_orient_) + ang_inc].theta;
				if(fake_laser_pose_[num_laser].theta > M_PI)
					fake_laser_pose_[num_laser].theta -= 2.0 * M_PI;
				if(fake_laser_pose_[num_laser].theta <= - M_PI)
					fake_laser_pose_[num_laser].theta += 2.0 * M_PI;

				fake_laser_pose_[num_laser].x = grid_pose_x;
				fake_laser_pose_[num_laser].y = grid_pose_y;

				//começa a varredura de cada theta-ésimo ponto do fake_laser
				passo = 0;
				int iteracao = range_max_fakelaser / passo_base; // 5.6 / 0.05 = 112

				for(int p = 1; p <= iteracao; p++)
				{
					//varredura do fake_laser
					passo = (passo_base * p); // + gaussian(0,laser_data_noise_);

					x = fake_laser_pose_[num_laser].x + (cos(fake_laser_pose_[num_laser].theta) * passo);
					y = fake_laser_pose_[num_laser].y + (sin(fake_laser_pose_[num_laser].theta) * passo);


					//arredondando os valores de x e y
					xi = x / res_;
					yi = y / res_;

					findObstacle(xi, yi);
					if (obstacle_finded_ == true)
					{
						fake_laser_data_[i][num_laser] = obstacle_;
						p = iteracao;
					}
					else
					{
						//não achou o obstáculo ou ele está a mais de "range_max" metros de distância
						fake_laser_data_[i][num_laser] = -999;
						passo = -9999;
					}
				}
				//passo = DISTANCIA FAKE DE CADA FEIXE DO LASER VIRTUAL!!!
				if(passo >= 0.0)
				{
					//somando todas as distâncias para cada giro de cada pose
					grid_pose_energy_[(i*qtdd_orient_) + ang_inc].sum += passo;
					num_energy_++;

					//Fazendo cálculo da energia (Ge)
					grid_pose_energy_[(i*qtdd_orient_) + ang_inc].energy += (1.0 - (passo / range_max_fakelaser));
				}
				else
				{
					grid_pose_energy_[(i*qtdd_orient_) + ang_inc].sum = -9999;
					grid_pose_energy_[(i*qtdd_orient_) + ang_inc].energy = -9999;
				}
			}
			//normalizando a energia
			grid_pose_energy_[(i*qtdd_orient_) + ang_inc].energy = grid_pose_energy_[(i*qtdd_orient_) + ang_inc].energy / qtdd_laser_;
		}
	}
	grids_ok_ = true;
	ordenaGrid();
}

void Filtro_Particulas_Sakldmcl::ordenaGrid()
{
	int grid_indice = 0;

	//carregando um vetor só com os indices dos valores positivos de energia (ie, quando cada feixe do fake_laser encontra o obstáculo)
	for(grid_indice = 0 ; grid_indice < (num_free_ * qtdd_orient_) ; grid_indice++)
	{
		if(grid_pose_energy_[grid_indice].energy >= 0.0)
		{
			grid_indice_sorted_[sorted_indice_] = grid_indice;
			sorted_indice_++;
		}
	}
	size_grid_energy_ = sorted_indice_;
	merge_sort( grid_pose_energy_, 0, (size_grid_energy_ - 1) );
}

void Filtro_Particulas_Sakldmcl::merge_sort (filtro_particulas_sakldmcl::grid_pose_energy vector[], const int low, const int high)
{
	int mid;
	if(low < high)
	{
		mid = (low + high) / 2;
		merge_sort( vector, low, mid );
		merge_sort( vector, mid + 1, high );
		merge( vector, low, mid, high );
	}
}

void Filtro_Particulas_Sakldmcl::merge (filtro_particulas_sakldmcl::grid_pose_energy vector[], const int low, const int mid, const int high)
{
	double * b = new double [high + 1 - low];
	int h,i,j,k;
	h = low;
	i = 0;
	j = mid + 1;

	while((h <= mid) && (j <= high))
	{
		//se o valor da energia da extrema esqueda for menor ou igual à energia do meio+1, o vetor b receberá o valor grid_indice
		if(vector[grid_indice_sorted_[h]].energy <= vector[grid_indice_sorted_[j]].energy)
		{
			b[i] = grid_indice_sorted_[h];
			h++;
		}
		else
		{
			b[i] = grid_indice_sorted_[j];
			j++;
		}
		i++;
	}
	if(h > mid)
	{
		for(k = j ; k <= high ; k++)
		{
			b[i] = grid_indice_sorted_[k];
			i++;
		}
	}
	else
	{
		for(k = h ; k <= mid ; k++)
		{
			b[i] = grid_indice_sorted_[k];
			i++;
		}
	}
	//Carrega o vetor com os indices ordenados
	for(k = 0 ; k <= high-low ; k++)
	{
		grid_indice_sorted_[k + low] = b[k];
	}
	delete[] b;
}

void Filtro_Particulas_Sakldmcl::calculoSER()
{
	//calcula a energia do robô no instante t dado os valores do laser
	if(calculo_SER_ok_ == false)
	{
		calculo_SER_loop_ = false;
		laser_data_energy_ = 0.0;

		for (int laser_num = 0 ; laser_num < qtdd_laser_ ; laser_num++)
		{
			if(laser_data_[laser_num] > 0.0)
			{
				laser_data_energy_ += (1 - (laser_data_[laser_num] / range_max_fakelaser));
			}else
			{
				cout<<"Atencao: Movimente ou gire o robo ate que o laser nao poduza mais dados NAN e INF"<<endl;
			}
		}
		laser_data_energy_ = laser_data_energy_ / qtdd_laser_;
		if(calculo_SER_loop_ == true)
		{
			calculo_SER_ok_ = false;
		}else calculo_SER_ok_ = true;
	}
}

bool Filtro_Particulas_Sakldmcl::buscaEnergiaSER()
{
	//busca uma energia igual à achada no calculoSER() no grid_pose_energy e salva o numero de particulas com a mesma energia.
	int esq, meio, dir;
	esq = 0;
	dir = size_grid_energy_ - 1;
	int t = 0;

	while (esq <= dir)
	{
		meio = (esq + dir) / 2;

		while ( (laser_data_energy_ - grid_pose_energy_[grid_indice_sorted_[meio]].energy) >= -ser_threshold_ && (laser_data_energy_ - grid_pose_energy_[grid_indice_sorted_[meio]].energy) <= ser_threshold_)
		{
			//cout<<"laser_data_energy_: "<<laser_data_energy_<<" | grid_pose_energy_[grid_indice_sorted_[meio]].energy: "<<grid_pose_energy_[grid_indice_sorted_[meio]].energy<<endl;
			int meio2 = meio;
			while(fabs(laser_data_energy_ - grid_pose_energy_[grid_indice_sorted_[meio]].energy) <= ser_threshold_)
			{
				indice_busca_SER_[t] = meio;
				meio--;
				t++;
			}
			while(fabs(laser_data_energy_ - grid_pose_energy_[grid_indice_sorted_[meio2 + 1]].energy) <= ser_threshold_)
			{
				indice_busca_SER_[t] = meio2 + 1;
				meio2++;
				t++;
			}
			num_particulas_SER_ = t;
			busca_energia_SER_ok_ = true;
			return true;
		}
		if ( (laser_data_energy_ - grid_pose_energy_[grid_indice_sorted_[meio]].energy) > ser_threshold_ ) esq = meio + 1;
		else dir = meio - 1;
	}

	return false;
}

//Daqui para baixo são os métodos usados para o KLD

void Filtro_Particulas_Sakldmcl::calculoNumKBins()
{
	//Método para carregar o vetor_pose_xy_bins_[] com as células intervaladas a cada bin.
	//zerar o vetor
	for(int j = 0 ; j < 10000 ; j++)
	{
		vetor_pose_xy_bins_[j] = 0;
	}

	k_bins_ = 0;

	int pose_x_pixel = 0;
	int pose_y_pixel = 0;

	int celula_pose_x = 0;
	int celula_pose_y = 0;

	for(int i = 0 ; i < num_part_ ; i++)
	{
		//pose em metros para pixels
		pose_x_pixel = particle_pose_[i].x / res_;
		pose_y_pixel = particle_pose_[i].y / res_;

		//relacionar os poses com os bins
		celula_pose_x = pose_x_pixel / bins_;
		celula_pose_y = pose_y_pixel / bins_;

		pose_xy_bins_ = (10000 * celula_pose_x) + celula_pose_y;

		//busca o valor de pose_xy_bins_
		buscaPosexyBins();
		//se existir, não faz nada. Se não, adicionar no início do vetor vetor_pose_xy_bins_,
		//incrementar o número de bins ocupados pelas partículas
		//e ordenar este vetor
		if(buscaPosexyBins() == false)
		{
			vetor_pose_xy_bins_[0] = pose_xy_bins_;
			k_bins_++;
			ordenaVetorPosexyBins();
		}
	}
}

void Filtro_Particulas_Sakldmcl::calculoSampleSize(int k)
{
	double a, b, c, x;
	int n;
	int num_part_ant = num_part_;

	if(num_part_ < num_min_part_)
	{
		num_min_part_ = num_part_;
	}

	if (k <= 1)
	{
		//ROS_INFO("k: %d !!! Não devia...", k);
	}
	//Cálculo do número de partículas utilizando a equação de KL-Distance
	else
	{
		a = 1;
		b = 2 / (9 * ((double) k - 1));
		c = sqrt(2 / (9 * ((double) k - 1))) * kld_z_;
		x = a - b + c;

		n = (int) ceil((k - 1) / (2 * kld_err_) * x * x * x);

		if (n < min_part_)
		{
			num_part_ = min_part_;
		}
		else if (n > max_part_)
		{
			num_part_ = max_part_;
		}
		else
		{
			num_part_ = n;
		}
		cout<<"Num_max_part: "<<max_part_<<" | Num_Part: "<<num_part_<<" | k_bins_: "<<k_bins_<<" | bins: "<<bins_<<" | kld_err: "<<kld_err_<<" | kld_z: "<<kld_z_<<endl;
		//printf("Number_Particle: %d | k_bins: %d | bins: %d | max_weight: %f | num_min_part: %d \n", num_part_, k_bins_, bins_,max_w_, num_min_part_);
	}
	if(num_part_ > num_part_ant)
	{
		int rand_indice;
		srand(time(NULL));
		for(int i = num_part_ant ; i < num_part_ ; i++)
		{
			rand_indice = rand() % num_part_ant; //random de 0 a num_part_ant
			particle_pose_[i].x = particle_pose_[rand_indice].x + gaussian(0.0, (3*move_noise_));
			particle_pose_[i].y = particle_pose_[rand_indice].y + gaussian(0.0, (3*move_noise_));
			particle_pose_[i].theta = particle_pose_[rand_indice].theta + gaussian(0.0, (3*turn_noise_));
		}
	}
	if(num_part_ < 1000)
		reduziu_num_part_ = true;
}

bool Filtro_Particulas_Sakldmcl::buscaPosexyBins()
{
	int esq, meio, dir;
	esq = 0;
	dir = 10000 - 1;

	while (esq <= dir)
	{
		meio = (esq + dir) / 2;
		if ( vetor_pose_xy_bins_[meio] == pose_xy_bins_)
		{
			return true;
		}
		if ( vetor_pose_xy_bins_[meio] < pose_xy_bins_) esq = meio + 1;
		else dir = meio - 1;
	}
	return false;
}

bool Filtro_Particulas_Sakldmcl::ordenaVetorPosexyBins()
{
	merge_sort( vetor_pose_xy_bins_, 0, (10000 - 1) );
	return true;
}

void Filtro_Particulas_Sakldmcl::merge_sort (int vector[], const int low, const int high)
{
	int mid;
	if(low < high)
	{
		mid = (low + high) / 2;
		merge_sort( vector, low, mid );
		merge_sort( vector, mid + 1, high );
		merge( vector, low, mid, high );
	}
}

void Filtro_Particulas_Sakldmcl::merge (int vector[], const int low, const int mid, const int high)
{
	double * b = new double [high + 1 - low];
	int h,i,j,k;
	h = low;
	i = 0;
	j = mid + 1;

	while((h <= mid) && (j <= high))
	{
		if(vetor_pose_xy_bins_[h] <= vetor_pose_xy_bins_[j])
		{
			b[i] = vetor_pose_xy_bins_[h];
			h++;
		}
		else
		{
			b[i] = vetor_pose_xy_bins_[j];
			j++;
		}
		i++;
	}
	if(h > mid)
	{
		for(k = j ; k <= high ; k++)
		{
			b[i] = vetor_pose_xy_bins_[k];
			i++;
		}
	}
	else
	{
		for(k = h ; k <= mid ; k++)
		{
			b[i] = vetor_pose_xy_bins_[k];
			i++;
		}
	}
	//Carrega o vetor com os indices ordenados
	for(k = 0 ; k <= high-low ; k++)
	{
		vetor_pose_xy_bins_[k + low] = b[k];
	}
	delete[] b;
}

void Filtro_Particulas_Sakldmcl::kidnapping()
{
	if(kidnapping_ok_ == true && desabilita_kid_ == false)
	{
		//habilita o espalhamento das partículas
		//cout<<"P_Local_: "<<num_part_local_<<" | P_Global_: "<<num_part_global_<<" | P_Total: "<<num_part_<<endl;
		//se peso máximo for menor que threshold, divide o sample set e habilita o cálculo de SER (compara fake_laser e real laser)

		if(desabilita_vary_sample_ == true)
		{
			num_part_local_ = alpha_sample_set_ * max_part_;
			num_part_global_ = max_part_ - num_part_local_;
			num_part_ = max_part_;
		}

		ROS_INFO("KIDNAPPING!!!!!");
		cout<<"weight: "<<max_w_before_norm_<<endl;
		calculo_SER_ok_ = false;
		busca_energia_SER_ok_ = false;
		reduziu_num_part_ = false;
		create_particle_ok_ = true;
		out_ = false;
		calc_new_num_particles_ = true;
	}
}

void Filtro_Particulas_Sakldmcl::escreveTxt()
{
	if(!myfile_.is_open())
	{
		cout<<"Nao conseguiu abrir dados.txt!!!"<<endl;
		create_particle_ok_ = false;
		zerar_deltas_ = false;
	}else iteration_++;

	time_bag_curr = ros::Time::now();
	time_bag_dif = time_bag_curr - time_bag_inicio;

	double x, y;

	//cout<<"time_bag_dif: "<<time_bag_dif<<endl;

/*	//Cada if abaixo grava o pose de cada posição desejada (no caso, 5 posições)
	if(time_bag_dif.toSec() > 20/r_bag_ && time_bag_dif.toSec() < 22/r_bag_ && pos_amostra_ == 1)
	{
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",";
		pos_amostra_++;
	}
	if(time_bag_dif.toSec() > 129.35/r_bag_ && time_bag_dif.toSec() < 132/r_bag_ && pos_amostra_ == 2)
	{
		//cout<<"x: "<<x<<" | pose_curr_msg.poses[0].position.x: "<<pose_curr_msg.poses[0].position.x<<endl;
		cout<<"time_bag_diff_1: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",";
		pos_amostra_++;
	}
	if(time_bag_dif.toSec() > 147.5/r_bag_ && time_bag_dif.toSec() < 150/r_bag_ && pos_amostra_ == 3)
	{
		cout<<"time_bag_diff_2: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",";
		pos_amostra_++;
	}
	if(time_bag_dif.toSec() > 169.2/r_bag_ && time_bag_dif.toSec() < 175/r_bag_ && pos_amostra_ == 4)
	{
		cout<<"time_bag_diff_3: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",";
		pos_amostra_++;
	}
	if(time_bag_dif.toSec() > 200.4/r_bag_ && time_bag_dif.toSec() < 205/r_bag_ && pos_amostra_ == 5)
	{
		cout<<"time_bag_diff_4: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",";
		pos_amostra_++;
	}
	if(time_bag_dif.toSec() > 222/r_bag_ && time_bag_dif.toSec() < 232/r_bag_ && pos_amostra_ == 6)
	{
		cout<<"time_bag_diff_5: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<endl;
		pos_amostra_++;
		num_bag_amostras_++;
	}
*/

	//Apenas para 1 posição. No caso foi usado para saber se o filtro convergiu errado ou não.
	//hall: 100
	//corner: 65
	//movimento: 95
	if(time_bag_dif.toSec() > 95/r_bag_ && time_bag_dif.toSec() < 98/r_bag_ && pos_amostra_ == 1)
	{
		//cout<<"x: "<<x<<" | pose_curr_msg.poses[0].position.x: "<<pose_curr_msg.poses[0].position.x<<endl;
		cout<<"time_bag_diff_1: "<<time_bag_dif.toSec()<<endl;
		x = 1.30 - pose_curr_msg.poses[0].position.x;
		y = 1.70 - pose_curr_msg.poses[0].position.y;
		myfile_<<x<<","<<y<<",iteration,"<<iteration_<<endl;
		pos_amostra_++;
		num_bag_amostras_++;
	}

/*	if(num_bag_amostras_ >= qtdd_amostras_)
	{
		if(max_part_ == 1000)
		{
			max_part_ = 2500;
		}else
		{
			max_part_ += 2500;
		}

		if(max_part_ > 7500)
		{
			if(desabilita_kld_ == false && desabilita_samcl_ == true)
			{
				myfile_.close();
				num_bag_amostras_ = -1;
			}
			if(desabilita_kld_ == true && desabilita_samcl_ == false)
			{
				//habilita KLD
				desabilita_kld_ = false;
				desabilita_samcl_ = true;
				max_part_ = 1000;
				ROS_INFO("INICIANDO/HABILITANDO KLD");
			}
			//se SA-KLD estiver habilitado,
			if(desabilita_kld_ == false && desabilita_samcl_ == false)
			{
				//habilita SAMCL
				desabilita_kld_ = true;
				max_part_ = 1000;
				ROS_INFO("INICIANDO/HABILITANDO SAMCL");
			}
		}
		num_bag_amostras_ = 0;
	}
*/
	//Para testar localização global de 500 a 9500 particulas
	if(num_bag_amostras_ >= qtdd_amostras_)
	{
		if(max_part_ < 9000)
		{
			max_part_ += 500;
		}else
		{
			myfile_.close();
			num_bag_amostras_ = -1;
		}
		num_bag_amostras_ = 0;
	}
	//cout<<"desabilita_kid: "<<desabilita_kid_<<endl;
}

void Filtro_Particulas_Sakldmcl::spin()
{
	ros::Rate loopRate(freq_);
	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();

/*		//Só para testar as distribuições normais
		for(int l=0;l<15;l++)
		{
			double turn = gaussian(0,turn_noise_);
			double move = gaussian(0,move_noise_);
			double laser = gaussian(0,laser_data_noise_);
			cout<<"turn: "<<turn<<" | move: "<<move<<" | laser: "<<laser<<endl;
		}
		cout<<"---------------------"<<endl;
		usleep(500000);
*/
/*		//Só para testar o gauss3
		for(int l=0;l<=5;l++)
		{
			double gauss3 = gaussian(l,laser_data_noise_,l);
			cout<<"gauss3: "<<gauss3<<endl;
		}
		cout<<"---------------------"<<endl;
		usleep(500000);
*/

		//verificando se o mapa foi carregado
		if (free_ok_ == true && occ_ok_ == true)
		{
			if(grids_ok_ == false)
			{
				//cria e ordena o grid do pose e energia
				//é feito apenas uma vez (depois que carrega um mapa)

				ros::Time inicio = ros::Time::now();
				createGrids();
				ros::Time fim = ros::Time::now();
				ros::Duration delta_t = fim - inicio;

				cout<<"Grids created. Time: "<<delta_t<<endl;
				//no início de tudo: num_part_ = max_part_
				if(desabilita_vary_sample_ == true)
				{
					num_part_ = max_part_;
					calc_new_num_particles_ = false;
				}

			}else if(grids_ok_ == true && odom_ok_ == true && laser_ok_ == true )
			{
				if(calc_new_num_particles_ == true)
				{
					calculo_SER_ok_ = false;
					busca_energia_SER_ok_ = false;
				}
				calculoSER();
				if(calculo_SER_ok_ == true) buscaEnergiaSER();
				if(busca_energia_SER_ok_ == false) calculo_SER_ok_ = false;

				if(desabilita_vary_sample_ == false && calc_new_num_particles_ == true && busca_energia_SER_ok_ == true) //num_particulas_SER_ != 0
				{
					calculoNumKBinsForCreateParticles();
					if(kidnapping_ok_ == true)
					{
						num_part_local_ = alpha_sample_set_ * num_part_;
						num_part_global_ = num_part_ - num_part_local_;
					}
				}

				//cout<<"max_w: "<<max_w_<<" | weight_threshold: "<<weight_threshold_<<endl;
				//cout<<"max_w_before: "<<max_w_before_norm_<<" | weight_threshold: "<<weight_threshold_<<endl;

				//haverá espalhamento de partículas se o peso máximo for menor que o threshold ou o número de partículas
				//aumentarem em função do fator_part_threshold. E, também, somente após a primeira convergência
				//se peso máximo for menor que threshold, divide o sample set e habilita o cálculo de SER (compara fake_laser e real laser)
				//if((max_w_ < weight_threshold_ || num_part_ > arredondamento) && prim_converg_ == true && reduziu_num_part_ == true)
				if(((max_w_before_norm_ < weight_threshold_) && prim_converg_ == true) || out_ == true)
				{
					if(desabilita_samcl_ == false && desabilita_kld_ == true)
					{
						kidnapping_ok_ = true;
						kidnapping();
					}
					if(desabilita_samcl_ == false && desabilita_kld_ == false && reduziu_num_part_ == true)
					{
						kidnapping_ok_ = true;
						kidnapping();
					}
					//cout<<"max_w: "<<max_w_<<" | weight_threshold: "<<weight_threshold_<<endl;

				}else if(kidnapping_ok_ == false)
				{
					num_part_local_ = num_part_;
					num_part_global_ = num_part_ - num_part_local_;
				}

				if(busca_energia_SER_ok_ == true)
				{
					if(zerar_deltas_ == false){num_part_local_ = 0;}
					createParticles();
					escreveTxt();

					if(create_particle_ok_ == false && zerar_deltas_ == false)
					{
						//zerando os deltas do pose
						pose_anterior_.x = pose_x_;
						pose_anterior_.y = pose_y_;
						pose_anterior_.theta = pose_theta_;

						zerar_deltas_ = true;
					}else if(create_particle_ok_ == false && zerar_deltas_ == true)
					{
							moveParticles();
					}
				}
			}
		}
	}
}

