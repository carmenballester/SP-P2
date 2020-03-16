#include "objectDetection.h"

void spawnViewer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud)
{
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
void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewer, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
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

    PointCloud<PointXYZRGBA>::Ptr scene (new PointCloud<PointXYZRGBA>);
    vector<PointCloud<PointXYZRGBA>::Ptr> objects;

    loadImages(scene, objects);
    // showCloud(scene);
    // showCloud(objects[0]);

    // Eliminar planos dominantes

    // Para la escena y cada uno de los objetos


        // Calcular keypoints

        // Calcular descriptores


    // Matching entre objetos y escena

    // Corregir malos emparejamientos

    // Calcular la transformacion

    // Refinar el resultado con ICP

  return 0;
}
