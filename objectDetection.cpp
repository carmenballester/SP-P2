#include "objectDetection.h"

void spawnViewer(PointCloud<PointXYZRGBA>::Ptr pointCloud/*, PointCloud<PointXYZRGBA>::Ptr cloudKP*/) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer->addCoordinateSystem (1.0);

	// Visualizar keypoints
	// for(size_t i=0; i<cloudKP->size(); i++){
	// 	string str = boost::lexical_cast<string>(i);
	// 	viewer->addSphere (cloudKP->points[i], 0.005, 0, 1, 0, str);
	// }


	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (pointCloud, rgb, "id0");
    while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

//Mostar nube y los keypoints
void showCloud(PointCloud<PointXYZRGBA>::Ptr& cloud/*, PointCloud<PointXYZRGBA>::Ptr& cloudKP*/) {
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewer, cloud/*, cloudKP*/);
	// cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

// Mostrar correspondencias
void spawnViewerCorr(PointCloud<PointXYZRGBA>::Ptr scene, PointCloud<PointXYZRGBA>::Ptr object, CorrespondencesPtr correspondences, PointCloud<PointXYZRGBA>::Ptr sceneKP,  PointCloud<PointXYZRGBA>::Ptr objectKP) {
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer ("3D Viewer"));

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbS(scene);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbO(object);
    viewer->addPointCloud<pcl::PointXYZRGBA> (scene, rgbS, "id0");
	viewer->addPointCloud<pcl::PointXYZRGBA> (object, rgbO, "id1");
	// Visualizar keypoints de la escena
	for(size_t i=0; i<sceneKP->size(); i++){
		string str = boost::lexical_cast<string>(i);
		viewer->addSphere (sceneKP->points[i], 0.005, 0, 1, 0, str);
	}

	// Visualizar keypoints del objeto
	for(size_t i=0; i<objectKP->size(); i++){
		string str = boost::lexical_cast<string>(i) + "a";
		viewer->addSphere (objectKP->points[i], 0.005, 0, 1, 0, str);
	}

	// Visualizar correspondencias
	viewer->addCorrespondences<PointXYZRGBA> (objectKP, sceneKP, *correspondences);
    while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

// Mostrar correspondencias
void showCorrespondences(PointCloud<PointXYZRGBA>::Ptr& scene, PointCloud<PointXYZRGBA>::Ptr& object, CorrespondencesPtr& correspondences,  PointCloud<PointXYZRGBA>::Ptr& sceneKP,  PointCloud<PointXYZRGBA>::Ptr& objectKP) {
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewerCorr, scene, object, correspondences, sceneKP, objectKP);
	// cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

// Mostrar transformacion
void spawnViewerTrans(PointCloud<PointXYZRGBA>::Ptr scene, PointCloud<PointXYZRGBA>::Ptr object) {
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer ("3D Viewer"));

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbS(scene);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> colorHandler(object, 255, 0, 0);

    viewer->addPointCloud<pcl::PointXYZRGBA> (scene, rgbS, "id0");
	viewer->addPointCloud<pcl::PointXYZRGBA> (object, colorHandler, "id1");

	viewer->addCoordinateSystem (1.0);
	while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

// Mostrar transformacion
void showTransformation(PointCloud<PointXYZRGBA>::Ptr& scene, PointCloud<PointXYZRGBA>::Ptr& object) {
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewerTrans, scene, object);
	// cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void transformation(CorrespondencesPtr correspondenceFilt, PointCloud<PointXYZRGBA>::Ptr object, PointCloud<PointXYZRGBA>::Ptr scene) {

	registration::TransformationEstimationSVD<PointXYZRGBA,PointXYZRGBA>::Matrix4 transform;
	registration::TransformationEstimationSVD<PointXYZRGBA, PointXYZRGBA> te;

	te.estimateRigidTransformation(*object, *scene, *correspondenceFilt, transform);

	// std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
	// printf ("\n");
	// printf ("    | %6.3f %6.3f %6.3f | \n", transform (0,0), transform (0,1), transform (0,2));
	// printf ("R = | %6.3f %6.3f %6.3f | \n", transform (1,0), transform (1,1), transform (1,2));
	// printf ("    | %6.3f %6.3f %6.3f | \n", transform (2,0), transform (2,1), transform (2,2));
	// printf ("\n");
	// printf ("t = < %0.3f, %0.3f, %0.3f >\n", transform (0,3), transform (1,3), transform (2,3));

	pcl::transformPointCloud(*object, *object, transform);
	showTransformation(scene, object);

//______________________________________________________________________________
	// IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA>::Matrix4 transform2;
	IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> icp;
	PointCloud<PointXYZRGBA>::Ptr object_registered;

	icp.setInputSource(object);
	icp.setInputTarget(scene);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);


	Eigen::Matrix4f transformation = icp.getFinalTransformation ();
	pcl::transformPointCloud(*object, *object, transformation);
	showTransformation(scene, object);

}

void rejection(CorrespondencesPtr correspondence, CorrespondencesPtr correspondenceFilt, PointCloud<PointXYZRGBA>::Ptr& object, PointCloud<PointXYZRGBA>::Ptr& scene, float maximumDistance) {

	registration::CorrespondenceRejectorDistance rejector;

	rejector.setInputSource<PointXYZRGBA>(object);
	rejector.setInputTarget<PointXYZRGBA>(scene);
	rejector.setInputCorrespondences(correspondence);
	rejector.setMaximumDistance(maximumDistance);
	rejector.getCorrespondences(*correspondenceFilt);

	cout << "Correspondences after filter: " <<correspondenceFilt->size () << endl;
}

void matching(CorrespondencesPtr correspondence, PointCloud<SHOT352>::Ptr& objectDes, PointCloud<SHOT352>::Ptr& sceneDes) { //kdTree

	//KDTREE
	// KdTreeFLANN<SHOT352> match_search;
	// match_search.setInputCloud(objectDes);
	//
	// //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	// for (size_t i=0; i<sceneDes->size(); i++) { // Si no funciona cambia los ++ de sitio
	//   vector<int> neigh_indices (1);
	//   vector<float> neigh_sqr_dists (1);
	//   if (!isfinite(sceneDes->at(i).descriptor[0])) { //skipping NaNs
	//     continue;
	//   }
	//   int found_neighs = match_search.nearestKSearch(sceneDes->at(i), 1, neigh_indices, neigh_sqr_dists);
	//   if(found_neighs == 1 && neigh_sqr_dists[0] < 0.7f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
	//   {
	//     Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
	//     correspondence->push_back (corr);
	//   }
	// }

	//CORRESPONDENCE ESTIMATION
	registration::CorrespondenceEstimation<SHOT352, SHOT352> est;
	est.setInputSource(objectDes);
	est.setInputTarget(sceneDes);
	est.determineCorrespondences (*correspondence, 0.8f);
}

void features(PointCloud<PointXYZRGBA>::Ptr& cloud, PointCloud<PointXYZRGBA>::Ptr& cloudKP, PointCloud<SHOT352>::Ptr& cloudDes) {

	// Normales
	PointCloud<Normal>::Ptr cloudNormal(new PointCloud<Normal> ());
	NormalEstimationOMP<PointXYZRGBA, Normal> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud);
	norm_est.compute(*cloudNormal);

	//SHOT
	SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT352> shot;

	shot.setRadiusSearch(0.05f);
	shot.setInputCloud(cloudKP);
	shot.setInputNormals(cloudNormal);
	shot.setSearchSurface(cloud);
	shot.compute(*cloudDes);

	// PFH
	// PointCloud<Normal>::Ptr cloudNormal(new PointCloud<Normal>);
	// NormalEstimation<PointXYZRGBA, Normal> normalEstimation;
	// PFHEstimation<PointXYZRGBA, Normal, PFHSignature125> pfh;
	// search::KdTree<PointXYZRGBA>::Ptr kdtree(new search::KdTree<PointXYZRGBA>);
	//
	// normalEstimation.setInputCloud(cloud);
	// normalEstimation.setRadiusSearch(0.03);
	// normalEstimation.setSearchMethod(kdtree);
	// normalEstimation.compute(*cloudNormal);
	//
	// pfh.setInputCloud(cloud);
	// pfh.setInputNormals(cloudNormal);
	// pfh.setSearchMethod(kdtree);
	// pfh.setRadiusSearch(0.05);
	// pfh.compute(*cloudDes);

	// Display
	cout << "Described: " << cloudDes->size() << endl;
}

void keypoints(PointCloud<PointXYZRGBA>::Ptr& cloud, PointCloud<PointXYZRGBA>::Ptr& cloudKP) { // Cambiar el tipo de cloudKP en función del método que se use
	// Uniform Sampling
	UniformSampling<PointXYZRGBA> uniform_sampling;

	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(0.05f);
	uniform_sampling.filter(*cloudKP);

	// ISS
	// ISSKeypoint3D<PointXYZRGBA, PointXYZRGBA> iss;
	// search::KdTree<PointXYZRGBA>::Ptr kdtree(new search::KdTree<PointXYZRGBA>);
	//
	// iss.setInputCloud(cloud);
	// iss.setSearchMethod(kdtree);
	// iss.setSalientRadius(6 * 0.005);
	// iss.setNonMaxRadius(4 * 0.005);
	// iss.setMinNeighbors(5);
	// iss.setThreshold21(0.975);
	// iss.setThreshold32(0.975);
	// iss.setNumberOfThreads(1);
	// iss.compute(*cloudKP);

	// Harris
	// HarrisKeypoint3D <PointXYZRGBA, PointXYZI> harris;
	//
  	// harris.setNonMaxSupression (true);
  	// harris.setInputCloud (cloud);
  	// harris.setThreshold (1e-6);
  	// harris.compute (*cloudKP);

	// Display
	// showCloud(cloud, cloudKP);
	cout << "Detected: " << cloudKP->size() << endl;
}

void getModel(PointCloud<PointXYZ>::Ptr scene, PointIndices::Ptr& inliers, float threshold) {

	// Definir el objeto que contiene los coeficientes del modelo
	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	// Definir el objeto con el que se va a segmentar (RANSAC)
	SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);
	// Definir que el modelo que se busca es un plano
	seg.setModelType (pcl::SACMODEL_PLANE);
	// Definir que se segmenta con RANSAC
	seg.setMethodType (pcl::SAC_RANSAC);
	// Definir el umbral de inliers
	seg.setDistanceThreshold (threshold);
	// Definir la nube que se va a segmentar
	seg.setInputCloud(scene);

	// Llamar al método que segmenta
	seg.segment(*inliers, *coefficients);

	// Comprobar si se ha encontrado un modelo
	if (inliers->indices.size () == 0) {
	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	    exit(-1);
  	}
}

void removePlane(PointCloud<PointXYZRGBA>::Ptr& scene, vector<float> threshold) {
	// Definir los puntos iniciales de la nube
	int pointsIn = (int)scene->points.size ();
	// Definir la nube en XYZ a la que le aplicamos RANSAC
	PointCloud<PointXYZ>::Ptr sceneXYZ(new PointCloud<PointXYZ>);
	//Definir el objeto para eliminar los puntos
	ExtractIndices<PointXYZRGBA> extract;

	for(size_t i=0; i<threshold.size(); i++) {
		// Copiar la escena a una nube de puntos XYZ
		copyPointCloud(*scene, *sceneXYZ);
		// Definir el objeto que contiene los inliers del modelo
		PointIndices::Ptr inliers(new PointIndices);

		// Obtener los inliers del plano
		getModel(sceneXYZ, inliers, threshold[i]);

		// Extraer de la escena los inliers obtenidos
		extract.setInputCloud(scene);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*scene);
	}
}

void loadImages(PointCloud<PointXYZRGBA>::Ptr& scene, vector<PointCloud<PointXYZRGBA>::Ptr>& objects) {

    string pcdFile;

    // Definir el archivo que contiene la escena
    pcdFile = "../clouds/scenes/snap_0point.pcd";

    // Cargar la nube de puntos de la escena
    if (pcl::io::loadPCDFile<PointXYZRGBA> (pcdFile, *scene) == -1) {
        cerr << "Error al cargar la nube de puntos " + pcdFile + "\n" << endl;
        exit(-1);
    }

    // Eliminar los valores NaN
    vector<int> indices;
    removeNaNFromPointCloud(*scene, *scene, indices);

    // Crear un vector con los nombres de todas las nubes de puntos de los objetos
    String folder = "../clouds/objects/*.pcd";
    vector<String> filenames;
    glob(folder, filenames);

    // Leer cada una de las nubes de puntos y almacenarla en el vector de nubes si se pueden leer correctamente
    for(size_t i=0; i<filenames.size(); i++) {
        PointCloud<PointXYZRGBA>::Ptr object (new PointCloud<PointXYZRGBA>);
        if (pcl::io::loadPCDFile<PointXYZRGBA> (filenames[i], *object) == -1) {
            cerr << "Error al cargar la nube de puntos " + pcdFile + "\n" << endl;
            exit(-1);
        }
		removeNaNFromPointCloud(*object, *object, indices);
        objects.push_back(object);
    }
}

int main (int argc, char** argv) {
	// Definir los umbrales para eliminar los planos dominantes
	// Inicializar el vector con el número de iteraciones y el valor del umbral para eliminar los planos dominantes
	// Se saca por empirismo, tanto los umbrales como el número de iteraciones
	vector<float> threshold;
	threshold.push_back(0.08);
	threshold.push_back(0.06);
	threshold.push_back(0.02);

	// Definir las nubes para la escena y los objetos
    PointCloud<PointXYZRGBA>::Ptr scene(new PointCloud<PointXYZRGBA>);
	vector<PointCloud<PointXYZRGBA>::Ptr> objects;

	// Definir los vectores de keypoints
	// Uniform Sampling - ISS
	PointCloud<PointXYZRGBA>::Ptr sceneKP(new PointCloud<PointXYZRGBA>);
	vector<PointCloud<PointXYZRGBA>::Ptr> objectsKP;
	for(size_t i=0; i<4; i++) {
		PointCloud<PointXYZRGBA>::Ptr object(new PointCloud<PointXYZRGBA>);
		objectsKP.push_back(object);
	}

	// Harris
	// PointCloud<PointXYZI>::Ptr sceneKP(new PointCloud<PointXYZI>);
	// vector<PointCloud<PointXYZI>::Ptr> objectsKP;
	// for(size_t i=0; i<4; i++) {
	// 	PointCloud<PointXYZI>::Ptr object(new PointCloud<PointXYZI>);
	// 	objectsKP.push_back(object);
	//  }

	// Definir los vectores de descriptores
	// SHOT
	PointCloud<SHOT352>::Ptr sceneDes(new PointCloud<SHOT352>);
	vector<PointCloud<SHOT352>::Ptr> objectsDes;
	for(size_t i=0; i<4; i++) {
		PointCloud<SHOT352>::Ptr object(new PointCloud<SHOT352>);
		objectsDes.push_back(object);
	}
	// PFH
	// PointCloud<PFHSignature125>::Ptr sceneDes(new PointCloud<PFHSignature125>());
	// vector<PointCloud<PFHSignature125>::Ptr> objectsDes;
	// for(size_t i=0; i<4; i++) {
	// 	PointCloud<PFHSignature125>::Ptr object(new PointCloud<PFHSignature125>);
	// 	objectsDes.push_back(object);
	// }

	// Normal
	// PointCloud<Normal>::Ptr sceneDes(new PointCloud<Normal>);
	// vector<PointCloud<Normal>::Ptr> objectsDes;
	// for(size_t i=0; i<4; i++) {
	// 	PointCloud<Normal>::Ptr object(new PointCloud<Normal>);
	// 	objectsDes.push_back(object);
	// }

	// Definir el vector de correspondencias
	vector<CorrespondencesPtr> correspondences;
	for(size_t i=0; i<4; i++) {
		CorrespondencesPtr correspondence(new Correspondences);
		correspondences.push_back(correspondence);
	}

	// Definir el vector de correspondencias filtradas y el vector que contiene los umbrales
	vector<float> maximumDistance;
	maximumDistance.push_back(1.1925);
	maximumDistance.push_back(1.21);
	maximumDistance.push_back(1.25);
	maximumDistance.push_back(1.2);
	vector<CorrespondencesPtr> correspondencesFilt;
	for(size_t i=0; i<4; i++) {
		CorrespondencesPtr correspondence(new Correspondences);
		correspondencesFilt.push_back(correspondence);
	}

	// Cargar la escena_________________________________________________________
    loadImages(scene, objects);

    // Eliminar planos dominantes_______________________________________________
	removePlane(scene, threshold);

    // Extraer los detectores y descriptores____________________________________
	keypoints(scene, sceneKP);
	for(int i=0; i<4; i++) {
		keypoints(objects[i],objectsKP[i]);
	}

	features(scene, sceneKP, sceneDes);
	for(int i=0; i<4; i++) {
		features(objects[i],objectsKP[i],objectsDes[i]);
	}

    //Matching entre objetos y escena__________________________________________
	for(int i=0; i<4; i++) {
		matching(correspondences[i],objectsDes[i],sceneDes);
		cout << "Correspondences found: " << correspondences[i]->size () << endl;
	}

    // Corregir malos emparejamientos___________________________________________
	for(int i=0; i<4; i++) {
		rejection(correspondences[i],correspondencesFilt[i],objects[i],scene, maximumDistance[i]);
	}

    // Calcular la transformacion y refinar el resultado con ___________________
	for(int i=0; i<4; i++) {
		transformation(correspondencesFilt[i],objects[i],scene);
	}

  return 0;
}
