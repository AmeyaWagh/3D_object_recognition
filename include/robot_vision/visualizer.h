#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

namespace visualizer{

	class BoundingBox{
	public:
	  visualization_msgs::Marker marker;
	  uint32_t shape;
	  
	  BoundingBox(std::string frame_id);
	  
	  visualization_msgs::Marker
	  getBoundingBox(float x,float y,float z,
	    float qx,float qy,float qz,float qw);

      visualization_msgs::Marker
      getBoundingBox(float minX,float minY,float minZ,
                     float maxX,float maxY,float maxZ,
                     std::string _color, int _id);
	};

} // namespace visualization

#endif
