/**
* @file         PlannerMonitor_LoadScenario.cpp
* @author       Tianyu Gu
* @date         04/06/2015
*/

#include "PlannerMonitor.h"
#include "ui_PlannerMonitor.h"

bool loadFromXMLFile(QString filename,
                     VehicleState_Def& vsDef,
                     vector<Lane_Def>& laneDef,
                     vector<Object_Def>& objDef) {
    QFile file( filename );
    if( file.open(QIODevice::ReadOnly) ) {
        log_info1("Succeeded Read File" << filename.toStdString());
        QXmlStreamReader xmlReader;
        xmlReader.setDevice(&file);;

        Lane_Def* tmpLaneDef = NULL;
        Object_Def* tmpObjDef = NULL;

        XML_ElementType typeMode = XML_Unknown;

        while (!xmlReader.isEndDocument()) {
            xmlReader.readNext();
            if (xmlReader.isEndElement()) {
                QStringRef name = xmlReader.name();
                // Switch mode
                if (name == "Scenario") { continue; }
                else if (name == "VehicleState") {
                    typeMode = XML_Unknown;
                    continue;
                }
                else if (name == "Lane") {
                    // Check
                    if(tmpLaneDef->numOfWaypoints != (int)tmpLaneDef->waypoints.size()) {
                        log_error1("XML_Lane: Inconsistent number of waypoints");
                    }

                    laneDef.push_back( *tmpLaneDef );
                    delete tmpLaneDef;
                    tmpLaneDef = NULL;

                    typeMode = XML_Unknown;
                    continue;
                }
                else if (name == "Object") {
                    objDef.push_back( *tmpObjDef );
                    delete tmpObjDef;
                    tmpObjDef = NULL;


                    typeMode = XML_Unknown;
                    continue;
                }
            }

            if (xmlReader.isStartElement()) {
                QStringRef name = xmlReader.name();
                // Switch mode
                if (name == "Scenario") {
                    continue;
                }
                else if (name == "VehicleState") {
                    typeMode = XML_VehicleState;
                    continue;
                }
                else if (name == "Lane") {
                    typeMode = XML_Lane;
                    if( tmpLaneDef != NULL ) delete tmpLaneDef;
                    tmpLaneDef = new Lane_Def;

                    continue;
                }
                else if (name == "Object") {
                    typeMode = XML_Object;
                    if( tmpObjDef != NULL ) delete tmpObjDef;
                    tmpObjDef = new Object_Def;

                    continue;
                }

                // Load mode values
                switch( typeMode ) {
                case XML_VehicleState:
                     if (name == "Latitude" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Latitude = atof( strElem.c_str() );
                     }
                     else if (name == "Longitude" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Longitude = atof( strElem.c_str() );
                     }
                     else if (name == "Heading" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Heading = atof( strElem.c_str() );
                     }
                     else if (name == "Curvature" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Curvature = atof( strElem.c_str() );
                     }
                     else if (name == "Speed" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Speed = atof( strElem.c_str() );
                     }
                     else if (name == "Acceleration" ) {
                         string strElem = xmlReader.readElementText().toStdString();
                         vsDef.Acceleration = atof( strElem.c_str() );
                     }
                     else {
                         log_error1("XML_VehicleState: Undefined Element");
                     }
                    break;
                case XML_Lane:
                    if (name == "Name" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpLaneDef->name = strElem;
                    }
                    else if (name == "NumOfWaypoints" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpLaneDef->numOfWaypoints = atoi( strElem.c_str() );
                    }
                    else if (name == "Waypoint" ) {
                        vector<double> num; num.clear();

                        string strElem = xmlReader.readElementText().toStdString();
                        string delimiter = ",";
                        size_t pos = 0;
                        string token;
                        while ((pos = strElem.find(delimiter)) != string::npos) {
                            token = strElem.substr(0, pos);
                            num.push_back( atof(token.c_str()) );
                            strElem.erase(0, pos + delimiter.length());
                        }
                        token = strElem;
                        num.push_back( atof(token.c_str()) );

                        tmpLaneDef->waypoints.push_back( make_pair(num[0], num[1]) );
                    }
                    else if (name == "Width" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpLaneDef->width_m = atof( strElem.c_str() );
                    }
                    else if (name == "SpeedLim" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpLaneDef->speedLim_mps = atof( strElem.c_str() );
                    }
                    else {
                        log_error1("XML_Lane: Undefined Element");
                    }
                    break;
                case XML_Object:
                    if (name == "Name" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->name = strElem;
                    }
                    else if (name == "Type" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        if( strElem == "Static" ) {
                            tmpObjDef->type = XML_Obj_Static;
                            break;
                        }
                        else if( strElem == "Car" ) {
                            tmpObjDef->type = XML_Obj_Car;
                            break;
                        }
                        else if( strElem == "Bicyclist") {
                            tmpObjDef->type = XML_Obj_Bicyclist;
                            break;
                        }
                        else if( strElem == "Pedestrian") {
                            tmpObjDef->type = XML_Obj_Pedestrian;
                            break;
                        }
                        else {
                            tmpObjDef->type = XML_Obj_Static;
                        }
                    }
                    else if (name == "Length" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->length_m = atof( strElem.c_str() );
                    }
                    else if (name == "Width" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->width_m = atof( strElem.c_str() );
                    }
                    else if (name == "Radius" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->radius_m = atof( strElem.c_str() );
                    }
                    else if (name == "Latitude" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->Latitude = atof( strElem.c_str() );
                    }
                    else if (name == "Longitude" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->Longitude = atof( strElem.c_str() );
                    }
                    else if (name == "Heading" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->Heading = atof( strElem.c_str() );
                    }
                    else if (name == "Speed" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->Speed = atof( strElem.c_str() );
                    }
                    else if (name == "LaneTracking" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->laneTracking = strElem.c_str();
                    }
                    else if (name == "LaneToTrack" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->laneToTrack = strElem.c_str();
                    }
                    else if (name == "TriggerId_Sim" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->triggerId_Sim = atoi( strElem.c_str() );
                    }
                    else if (name == "TriggerId_SwitchLane" ) {
                        string strElem = xmlReader.readElementText().toStdString();
                        tmpObjDef->triggerId_SwitchLane = atoi( strElem.c_str() );
                    }
                    break;
                case XML_Unknown:
                    log_error1("XML encountered weird element..." << xmlReader.readElementText().toStdString() );
                default:
                    continue;
                }
            }
        }

        return true;
    }
    else {
        log_error1("Failed Read File: " << filename.toStdString());
        return false;
    }
}


void PlannerMonitor::on_btnLoadScenarioFromFile_clicked()
{
    QString filename = QFileDialog::getOpenFileName( this,
                                                     tr("Open File"),
                                                     QString::fromStdString(string(getenv("DIR_MY_SIM_SCENARIO"))),
                                                     "XML File (*.xml)" );

    VehicleState_Def vsDef;
    vector<Lane_Def> laneDef;
    vector<Object_Def> objDef;
    loadFromXMLFile( filename, vsDef, laneDef, objDef );

    // ToDo: removed to directly accept vsDef, laneDefs, objDefs
    WorldModel* wm = new WorldModel;
    wm->initVehicleState_Def( vsDef );
    wm->initRoadModel_Def( laneDef );
    wm->initObject_Def( objDef,
                        &trigger0_, &trigger1_, &trigger2_ );

    sim_->setWorldModel( wm );
}
