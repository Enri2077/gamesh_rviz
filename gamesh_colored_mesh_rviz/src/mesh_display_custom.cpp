/*
 * Copyright TODO
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreMaterialManager.h>

#include "rviz/display_context.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include "mesh_display_custom.h"

namespace rviz {

ColoredMeshDisplayCustom::ColoredMeshDisplayCustom() :
		Display(), mesh_node_(NULL), manual_object_(NULL), lightGV_(NULL), initialized_(
				false) {
	std::cout << "ColoredMeshDisplayCustom" << std::endl;

	mesh_topic_property_ = new RosTopicProperty("Mesh Topic", "",
			QString::fromStdString(
//					ros::message_traits::datatype<shape_msgs::Mesh>()),
					ros::message_traits::datatype<gamesh_bridge::GameshMesh>()),
			"shape_msgs::Mesh topic to subscribe to.", this,
			SLOT(updateTopic()));

	mesh_alpha_property_ = new FloatProperty("Mesh Alpha", 1.0f,
			"Amount of transparency for the mesh.", this,
			SLOT(updateMeshProperties()));

	mesh_color_property_ = new ColorProperty("Mesh Color",
			QColor(152, 152, 152), "Useless property.", this,
			SLOT(updateMeshProperties()));

	light_position_property_ = new VectorProperty("Light Position",
			Ogre::Vector3::ZERO, "position of the light source in /world", this,
			SLOT(updateLightProperties()));

	light_diffuse_color_property_ =
			new ColorProperty("Light Diffuse Color", QColor(255, 0, 0),
					"The color the light source emits, that illuminates the mesh in a diffused and opaque way.",
					this, SLOT(updateLightProperties()));

	light_specular_color_property_ =
			new ColorProperty("Light Specular Color", QColor(225, 50, 50),
					"The color the light source emits, that illuminates the mesh in a specular and glossy way.",
					this, SLOT(updateLightProperties()));

}

ColoredMeshDisplayCustom::~ColoredMeshDisplayCustom() {
	std::cout << "~ColoredMeshDisplayCustom" << std::endl;
	unsubscribe();
}

void ColoredMeshDisplayCustom::onInitialize() {
	std::cout << "onInitialize" << std::endl;
	Display::onInitialize();

	context_->getSceneManager()->addRenderQueueListener(this);

}

void ColoredMeshDisplayCustom::updateMesh(
//		const shape_msgs::Mesh::ConstPtr& mesh) {
		const gamesh_bridge::GameshMesh::ConstPtr& mesh) {
	std::cout << "updateMesh" << std::endl;
	boost::mutex::scoped_lock lock(mesh_mutex_);

	// create our scenenode and material
	load();

	// set properties

	if (!manual_object_) {
		static uint32_t count = 0;
		std::stringstream ss;
		ss << "MeshObject" << count++;
		manual_object_ = context_->getSceneManager()->createManualObject(
				ss.str());
		mesh_node_->attachObject(manual_object_);
	}

	// If we have the same number of tris as previously, just update the object
	if (last_mesh_.vertices.size() > 0
			&& mesh->vertices.size() * 2 == last_mesh_.vertices.size()) {
		manual_object_->beginUpdate(0);
	} else // Otherwise clear it and begin anew
	{
		manual_object_->clear();
		manual_object_->estimateVertexCount(mesh->vertices.size() * 2);
		manual_object_->begin(mesh_material_->getName(),
				Ogre::RenderOperation::OT_TRIANGLE_LIST);
	}

	const std::vector<geometry_msgs::Point>& points = mesh->vertices;
	const std::vector<std_msgs::ColorRGBA>& colours = mesh->vertex_colors;


	for (auto& triangle : mesh->triangles) {
//	for (size_t i = 0; i < mesh->triangles.size(); i++) {
//		auto& triangle = mesh->triangles[i];

		// make sure we have front-face/back-face triangles
//		for (int side = 0; side < 2; side++) {
		std::vector<Ogre::Vector3> corners(3);

		for (size_t cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
			auto& vertex = points[triangle.vertex_indices[cornerIndex]];

			corners[cornerIndex] = Ogre::Vector3(vertex.x, vertex.y, vertex.z);
		}

		Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(
				corners[2] - corners[0]);
		normal.normalise();

		for (size_t cornerIndex = 0; cornerIndex < 3; cornerIndex++) {
			auto& colour = colours[triangle.vertex_indices[cornerIndex]];

			manual_object_->position(corners[cornerIndex]);
//				manual_object_->colour(mesh_color_property_->getColor().redF(),
//						mesh_color_property_->getColor().greenF(),
//						mesh_color_property_->getColor().blueF());
			manual_object_->colour(colour.r, colour.g, colour.b);
			manual_object_->normal(normal);
		}
//		}

//		// make sure we have front-face/back-face triangles
//		for (int side = 0; side < 2; side++) {
//			std::vector<Ogre::Vector3> corners(3);
//
//			for (size_t c = 0; c < 3; c++) {
//				size_t corner = side ? 2 - c : c; // order of corners if side == 1
//				corners[corner] = Ogre::Vector3(
//						points[mesh->triangles[i].vertex_indices[corner]].x,
//						points[mesh->triangles[i].vertex_indices[corner]].y,
//						points[mesh->triangles[i].vertex_indices[corner]].z);
//			}
//
//			Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(
//					corners[2] - corners[0]);
//			normal.normalise();
//
//			for (size_t c = 0; c < 3; c++) {
//				manual_object_->position(corners[c]);
////				manual_object_->colour(mesh_color_property_->getColor().redF(),
////						mesh_color_property_->getColor().greenF(),
////						mesh_color_property_->getColor().blueF());
//				manual_object_->colour(mesh->vertex_colors[0].r, , );
//				manual_object_->normal(normal);
//			}
//		}

	}

	manual_object_->end();

	mesh_material_->setCullingMode(Ogre::CULL_CLOCKWISE);

	last_mesh_ = *mesh;
}

void ColoredMeshDisplayCustom::updateLightProperties() {
	std::cout << "updateLightProperties" << std::endl;

	if (!initialized_)
		return;

	if (!context_->getSceneManager()->hasLight("LightGV")) {
		lightGV_ = context_->getSceneManager()->createLight("LightGV");
		lightGV_->setType(Ogre::Light::LT_POINT);
		lightGV_->setAttenuation(100, 1.0, 0.045, 0.0075);
		lightGV_->setCastShadows(true);
	}

	lightGV_->setPosition(light_position_property_->getVector().x,
			light_position_property_->getVector().y,
			light_position_property_->getVector().z);

	lightGV_->setDiffuseColour(light_diffuse_color_property_->getColor().redF(),
			light_diffuse_color_property_->getColor().greenF(),
			light_diffuse_color_property_->getColor().blueF());

	lightGV_->setSpecularColour(
			light_specular_color_property_->getColor().redF(),
			light_specular_color_property_->getColor().greenF(),
			light_specular_color_property_->getColor().blueF());

}

void ColoredMeshDisplayCustom::updateMeshProperties() {
	std::cout << "updateMeshProperties" << std::endl;

	if (!initialized_)
		return;

	// update color/alpha
	Ogre::Technique* technique = mesh_material_->getTechnique(0);
	Ogre::Pass* pass = technique->getPass(0);

	// Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, mesh_alpha_property_->getFloat());
	// pass->setSelfIllumination(self_illumination_color);

	// Ogre::ColourValue diffuse_color(mesh_color_property_->getColor().redF(), mesh_color_property_->getColor().greenF(), mesh_color_property_->getColor().blueF(), mesh_alpha_property_->getFloat());
	// pass->setDiffuse(diffuse_color);
	//
	// Ogre::ColourValue ambient_color(mesh_color_property_->getColor().redF()/2.0f, mesh_color_property_->getColor().greenF()/2.0f, mesh_color_property_->getColor().blueF()/2.0f, mesh_alpha_property_->getFloat());
	// pass->setAmbient(ambient_color);
	//
	// Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
	// pass->setSpecular(specular_color);
	//
	// Ogre::Real shininess = 64.0f;
	// pass->setShininess(shininess);

	// pass->setVertexColourTracking(Ogre::TVC_DIFFUSE | Ogre::TVC_AMBIENT | Ogre::TVC_SPECULAR);

	// pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	// pass->setDepthWriteEnabled(false);

	context_->queueRender();
}

void ColoredMeshDisplayCustom::updateTopic() {
	std::cout << "updateTopic" << std::endl;
	unsubscribe();
	subscribe();
}

void ColoredMeshDisplayCustom::subscribe() {
	std::cout << "subscribe" << std::endl;
	if (!isEnabled()) {
		return;
	}

	if (!mesh_topic_property_->getTopic().isEmpty()) {
		try {
			mesh_sub_ = nh_.subscribe(mesh_topic_property_->getTopicStd(), 1,
					&ColoredMeshDisplayCustom::updateMesh, this);
			setStatus(StatusProperty::Ok, "Topic", "OK");
		} catch (ros::Exception& e) {
			setStatus(StatusProperty::Error, "Topic",
					QString("Error subscribing: ") + e.what());
		}
	}

}

void ColoredMeshDisplayCustom::unsubscribe() {
	std::cout << "unsubscribe" << std::endl;
//	Display::unsubscribe();
	mesh_sub_.shutdown(); // TODO ?
}

void ColoredMeshDisplayCustom::load() {
	std::cout << "load" << std::endl;
	if (mesh_node_ != NULL)
		return;

	initialized_ = true;
	static int count = 0;
	std::stringstream ss;
	ss << "MeshNode" << count++ << "Group";
	Ogre::MaterialManager& material_manager =
			Ogre::MaterialManager::getSingleton();
	Ogre::String resource_group_name = ss.str();

	Ogre::ResourceGroupManager& rg_mgr =
			Ogre::ResourceGroupManager::getSingleton();

	Ogre::String material_name = resource_group_name + "MeshMaterial";

	if (!rg_mgr.resourceGroupExists(resource_group_name)) {
		rg_mgr.createResourceGroup(resource_group_name);

		mesh_material_ = material_manager.create(material_name,
				resource_group_name);
		initialized_ = true;

		Ogre::Technique* technique = mesh_material_->getTechnique(0);
		Ogre::Pass* pass = technique->getPass(0);

		// Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, mesh_alpha_property_->getFloat());
		// pass->setSelfIllumination(self_illumnation_color);

		// Ogre::ColourValue diffuse_color(mesh_color_property_->getColor().redF(), mesh_color_property_->getColor().greenF(), mesh_color_property_->getColor().blueF(), mesh_alpha_property_->getFloat());
		// pass->setDiffuse(diffuse_color);
		//
		// Ogre::ColourValue ambient_color(mesh_color_property_->getColor().redF()/2.0f, mesh_color_property_->getColor().greenF()/2.0f, mesh_color_property_->getColor().blueF()/2.0f, mesh_alpha_property_->getFloat());
		// pass->setAmbient(ambient_color);
		//
		// Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
		// pass->setSpecular(specular_color);
		//
		// Ogre::Real shininess = 64.0f;
		// pass->setShininess(shininess);

		pass->setVertexColourTracking(
				Ogre::TVC_DIFFUSE | Ogre::TVC_AMBIENT | Ogre::TVC_SPECULAR);

		pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

		// mesh_material_->setCullingMode(Ogre::CULL_CLOCKWISE);
	}

	mesh_node_ = this->scene_node_->createChildSceneNode();
}

void ColoredMeshDisplayCustom::onEnable() {
	std::cout << "onEnable" << std::endl;
	subscribe();
}

void ColoredMeshDisplayCustom::onDisable() {
	std::cout << "onDisable" << std::endl;
	unsubscribe();
}

//void ColoredMeshDisplayCustom::update(float wall_dt, float ros_dt) {
//	// std::cout << "update" << std::endl;
////    time_since_last_transform_ += wall_dt;
//
////    // just added automatic rotation to make it easier  to test things
////    if(projector_node_ != NULL)
////    {
////        projector_node_->rotate(Ogre::Vector3::UNIT_Y, Ogre::Degree(wall_dt * 50));
////        rotation_property_->setQuaternion(projector_node_->getOrientation());
////    }
//
////    if( !topic_property_->getTopic().isEmpty() )
////    {
////        std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());
////        if(caminfo_sub_.getTopic().compare(caminfo_topic) != 0)
////        {
////            //std::cout<<"updating topic" <<std::endl;
////
////            caminfo_sub_.unsubscribe();
////            try
////            {
////                caminfo_sub_.subscribe( update_nh_, caminfo_topic, 1 );
////                // std::cout<<"The subscription happens"<<std::endl;
////                setStatus( StatusProperty::Ok, "Camera Info", "OK" );
////            }
////            catch( ros::Exception& e )
////            {
////                setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
////            }
////        }
////
////        try
////        {
////            updateCamera(texture_.update());
////        }
////        catch( UnsupportedImageEncoding& e )
////        {
////            setStatus(StatusProperty::Error, "Image", e.what());
////        }
////    }
//}

void ColoredMeshDisplayCustom::clear() {
	std::cout << "clear" << std::endl;
	context_->queueRender();

}

void ColoredMeshDisplayCustom::reset() {
	std::cout << "reset" << std::endl;
	Display::reset();
	clear();
}

void ColoredMeshDisplayCustom::updateQueueSize() {
	std::cout << "updateQueueSize" << std::endl;
}
//
//void ColoredMeshDisplayCustom::fixedFrameChanged() {
//	std::cout << "fixedFrameChanged" << std::endl;
//	Display::fixedFrameChanged();
//}

}// namespace rviz
