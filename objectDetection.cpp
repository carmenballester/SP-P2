#include "objectDetection.h"

void spawnViewer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->addCoordinateSystem (1.0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (pointCloud, rgb, "id0");
    while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

//Mostar nube
void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud) {
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewer, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void keypoints(PointCloud<PointXYZRGBA>::Ptr& cloud, PointCloud<PointXYZRGBA>::Ptr& cloudKP, PointCloud<SHOT352>::Ptr& cloudDes) {
	// Detector de keypoints
	UniformSampling<PointXYZRGBA> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(0.05f);
	uniform_sampling.filter(*cloudKP);
	// Normales
	PointCloud<Normal>::Ptr cloudNormal(new PointCloud<Normal> ());
	NormalEstimationOMP<PointXYZRGBA, Normal> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud);
	norm_est.compute (*cloudNormal);

	//Descriptor
	pcl::SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT352> descr_est;
	descr_est.setRadiusSearch(0.2f);
	descr_est.setInputCloud(cloudKP);
	descr_est.setInputNormals(cloudNormal);
	descr_est.setSearchSurface(cloud);
	descr_est.compute(*cloudDes);
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
        objects.push_back(object);
    }

}

int main (int argc, char** argv) {

	// Definir las nubes para la escena y sus keypoints y el descriptor
    PointCloud<PointXYZRGBA>::Ptr scene(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr sceneKP(new PointCloud<PointXYZRGBA>);
	PointCloud<SHOT352>::Ptr sceneDes(new PointCloud<SHOT352>);

	// Definir los vectores de nubes para los objetos y sus keypoints y los descriptores
	vector<PointCloud<PointXYZRGBA>::Ptr> objects;
	vector<PointCloud<PointXYZRGBA>::Ptr> objectsKP;
	vector<PointCloud<SHOT352>::Ptr> objectsDes;
	for(size_t i=0; i<4; i++) {
		PointCloud<PointXYZRGBA>::Ptr object(new PointCloud<PointXYZRGBA>);
		objectsKP.push_back(object);
	}
	for(size_t i=0; i<4; i++) {
		PointCloud<SHOT352>::Ptr object(new PointCloud<SHOT352>);
		objectsDes.push_back(object);
	}

	// Definir los umbrales para eliminar los planos dominantes
	// Inicializar el vector con el número de iteraciones y el valor del umbral para eliminar los planos dominantes
	// Se saca por empirismo, tanto los umbrales como el número de iteraciones
	vector<float> threshold;
	threshold.push_back(0.08);
	threshold.push_back(0.04);
	threshold.push_back(0.02);

	// Cargar la escena_________________________________________________________
    loadImages(scene, objects);

    // Eliminar planos dominantes_______________________________________________
	removePlane(scene, threshold);

    // Extraer los detectores y descriptores____________________________________
	cout << "escena" << endl;
	keypoints(scene, sceneKP, sceneDes);
	for(int i=0; i<4; i++) {
		cout << "objeto " << i << endl;
		keypoints(objects[i],objectsKP[i],objectsDes[i]);
	}
	
    // Matching entre objetos y escena

    // Corregir malos emparejamientos

    // Calcular la transformacion

    // Refinar el resultado con ICP

  return 0;
}
