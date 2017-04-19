#include "file_io.h"
#include "basic_types.h"

namespace FileIO
{
	Sample* load_point_cloud_file( std::string filename, FILE_TYPE type, IndexType sample_idx )
	{
// 		FILE* in_file = fopen(filename.c_str(), "r");
// 
// 		if(in_file==NULL)
// 			return nullptr;
// 
// 		Sample* new_sample = new Sample;
// //		ScalarType vx0,vy0,vz0;
// // 		bool first_vertex = true;
// 		while ( true )
// 		{
// 
// 			ScalarType vx,vy,vz,nx,ny,nz,cx,cy,cz;
// 			int status = fscanf(in_file, "%f %f %f %f %f %f %f %f %f\n",&vx,&vy,&vz,
// 				&nx,&ny,&nz,
// 				&cx,&cy,&cz);
// 			if(status==EOF)break;
// // 			if ( first_vertex )
// // 			{
// // 				vx0 = vx,vy0 = vy, vz0 = vz;
// // 				PointType v(0.,0.,0.);
// // 				ColorType cv(cx/255.,cy/255.,cz/255.,1.);
// // 				NormalType nv(nx,ny,nz);
// // 				new_sample->add_vertex(v, nv, cv);
// // 				first_vertex = false;
// // 				continue;
// // 			}
// // 			PointType v(vx - vx0,vy - vy0,vz - vz0);
// 			PointType v(vx,vy,vz);
// 			ColorType cv(cx/255.,cy/255.,cz/255.,1.);
// 			NormalType nv(nx,ny,nz);
// 
// 			new_sample->add_vertex(v, nv, cv);
// 		
// 
// 		}
// 
// 		//give the new sample a color
// 		new_sample->set_visble(false);
// 		new_sample->set_color( Color_Utility::span_color_from_table( sample_idx ) );
// 		new_sample->build_kdtree();
// 		return new_sample;

		//read .xyz or .ply files
		FILE* in_file = fopen(filename.c_str(), "r");

		if(in_file==NULL)
			return nullptr;

		Sample* new_sample = new Sample;
		//		ScalarType vx0,vy0,vz0;
		// 		bool first_vertex = true;

		if( type == FileIO::XYZ)
		{
			while ( true )
			{

				ScalarType vx,vy,vz,nx,ny,nz,cx,cy,cz;
				int status = fscanf(in_file, "%f %f %f %f %f %f %f %f %f\n",&vx,&vy,&vz,
					&nx,&ny,&nz,
					&cx,&cy,&cz);
				if(status==EOF)break;
				// 			if ( first_vertex )
				// 			{
				// 				vx0 = vx,vy0 = vy, vz0 = vz;
				// 				PointType v(0.,0.,0.);
				// 				ColorType cv(cx/255.,cy/255.,cz/255.,1.);
				// 				NormalType nv(nx,ny,nz);
				// 				new_sample->add_vertex(v, nv, cv);
				// 				first_vertex = false;
				// 				continue;
				// 			}
				// 			PointType v(vx - vx0,vy - vy0,vz - vz0);
				PointType v(vx,vy,vz);
				ColorType cv(cx/255.,cy/255.,cz/255.,1.);
				NormalType nv(nx,ny,nz);

				new_sample->add_vertex(v, nv, cv);


			}

		}else if( type == FileIO::PLY)
		{
			int vcount=0;
			int fcount=0;
// 			fscanf(in_file,"ply\nformat ascii 1.0\nelement vertex %d\n",&vcount);
// 			fscanf(in_file,"property float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n");
// 			fscanf(in_file,"end_header\n");

			//born-shihao
			fscanf(in_file,"ply\nformat ascii 1.0\ncomment PCL generated\nelement vertex %d\n",&vcount);
			fscanf(in_file,"property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\nproperty float ny\nproperty float nz\n");
			fscanf(in_file,"property float curvature\nelement camera 1\nproperty float view_px\nproperty float view_py\nproperty float view_pz\nproperty float x_axisx\nproperty float x_axisy\nproperty float x_axisz\n");
			fscanf(in_file,"property float y_axisx\nproperty float y_axisy\nproperty float y_axisz\nproperty float z_axisx\nproperty float z_axisy\nproperty float z_axisz\n");
			fscanf(in_file,"property float focal\nproperty float scalex\nproperty float scaley\nproperty float centerx\nproperty float centery\nproperty int viewportx\nproperty int viewporty\nproperty float k1\nproperty float k2\n");
			fscanf(in_file,"end_header\n");

			IndexType vIdx = 0;

			while(vIdx++ < vcount)
			{

				ScalarType vx,vy,vz,nx,ny,nz,cx,cy,cz;
				IndexType non;
// 				int status = fscanf(in_file, "%f %f %f %f %f %f %f %f %f\n",&vx,&vy,&vz,
// 					&nx,&ny,&nz,
// 					&cx,&cy,&cz);
// 				if(status==EOF)break;

				//born
				int status = fscanf(in_file, "%f %f %f %f %f %f %f %f %f %d\n",
					&vx,&vy,&vz,
					&cx,&cy,&cz,
					&nx,&ny,&nz,
					&non);
				if(status==EOF)break;

				PointType v(vx,vy,vz);
				ColorType cv(cx/255.,cy/255.,cz/255.,1.);
				NormalType nv(nx,ny,nz);

				new_sample->add_vertex(v, nv, cv);

				//++ vIdx;
//No color
				//ScalarType vx,vy,vz,nx,ny,nz,cx,cy,cz;
				//int status = fscanf(in_file, "%f %f %f %f %f %f\n",&vx,&vy,&vz,
				//	&nx,&ny,&nz);
				//if(status==EOF)break;

				//PointType v(vx,vy,vz);
				//ColorType cv(239/255.,122/255.,47/255.,1.);
				//NormalType nv(nx,ny,nz);

				//new_sample->add_vertex(v, nv, cv);
			}

		}else{
			return NULL;
		}

		fclose(in_file);


		//give the new sample a color
		new_sample->set_visble(false);
		new_sample->set_color( Color_Utility::span_color_from_table( sample_idx ) );
		new_sample->build_kdtree();

		//add by huayun
		Box box = new_sample->getBox();
		ScalarType dia_distance = box.diag();
		return new_sample;


	}
}