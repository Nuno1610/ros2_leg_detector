/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <leg_detector/laser_processor.h>

namespace laser_processor 
{
    Sample* Sample::Extract(int ind, const sensor_msgs::msg::LaserScan& scan){

        Sample* s = new Sample();

        s->index = ind;
        s->range = scan.ranges[ind];
        s->x = cos( scan.angle_min + ind*scan.angle_increment ) * s->range;
        s->y = sin( scan.angle_min + ind*scan.angle_increment ) * s->range;
        if (s->range > scan.range_min && s->range < scan.range_max)
        {
            return s;
        }
        else
        {
            delete s;
            return NULL;
        }
    }

    
    void SampleSet::clear(){

        for (SampleSet::iterator i = begin(); i != end(); ++i)
            delete (*i);

        std::set<Sample*, CompareSample>::clear();

    }

    geometry_msgs::msg::Point SampleSet::getPosition(){

        geometry_msgs::msg::Point point;
        float x_mean = 0.0;
        float y_mean = 0.0;
        for (iterator i = begin(); i != end(); ++i)
        {
            x_mean += ((*i)->x)/size();
            y_mean += ((*i)->y)/size();
        }
        point.x = x_mean; 
        point.y = y_mean;
        point.z = 0.0;
        return point;
    }

    ScanProcessor::ScanProcessor(const sensor_msgs::msg::LaserScan& scan){

        scan_ = scan;

        SampleSet* cluster = new SampleSet;

        for (unsigned long int i = 0; i < scan.ranges.size(); i++)
        {
            Sample* s = Sample::Extract(i, scan);

            if (s != NULL)
            cluster->insert(s);
        }

        clusters_.push_back(cluster);

    }

    ScanProcessor::~ScanProcessor(){
        
        for ( std::list<SampleSet*>::iterator c = clusters_.begin(); c != clusters_.end(); ++c)
            delete (*c);

    }

    void ScanProcessor::removeLessThan(uint32_t num){

        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        while (c_iter != clusters_.end())
        {
            if ( (*c_iter)->size() < num )
            {
                delete (*c_iter);
                clusters_.erase(c_iter++);
            } 
            else 
            {
                ++c_iter;
            }
        }
    }

    void ScanProcessor::splitConnected(float thresh){

        // Contiene temporalmente los clústeres divididos,
        // ya que la lista existente se modificará durante el proceso.
        std::list<SampleSet*> tmp_clusters;

        std::list<SampleSet*>::iterator c_iter = clusters_.begin();

        while (c_iter != clusters_.end())
        {

            while ((*c_iter)->size() > 0)
            {

                // Recorre las muestras del escaneo láser en clusters_,
                // recopila las que están a una distancia euclídea menor que <thresh>
                // y guarda los nuevos clústeres en tmp_clusters.
                SampleSet::iterator s_first = (*c_iter)->begin();
                std::list<Sample*> sample_queue;
                sample_queue.push_back(*s_first);
                (*c_iter)->erase(s_first);
                std::list<Sample*>::iterator s_q = sample_queue.begin();

                while (s_q != sample_queue.end()){

                    int expand = (int)(asin( thresh / (*s_q)->range ) / scan_.angle_increment);

                    SampleSet::iterator s_rest = (*c_iter)->begin();

                    while ( (s_rest != (*c_iter)->end() and (*s_rest)->index < (*s_q)->index + expand ) ){

                        if (sqrt( pow( (*s_q)->x - (*s_rest)->x, 2.0f) + pow( (*s_q)->y - (*s_rest)->y, 2.0f)) < thresh){

                            sample_queue.push_back(*s_rest);
                            (*c_iter)->erase(s_rest++);
                        }
                        else {
                            ++s_rest;
                        }    
                    }
                    s_q++;
                }
                // Mueve todas las muestras al nuevo clúster.
                SampleSet* c = new SampleSet;
                for (s_q = sample_queue.begin(); s_q != sample_queue.end(); s_q++)
                    c->insert(*s_q);

                // Guarda los clústeres temporales.
                tmp_clusters.push_back(c);
            }

            // Ahora que c_iter está vacío, se puede eliminar.
            delete (*c_iter);

            // Y lo elimina de la lista de clústeres.
            clusters_.erase(c_iter++);
        }

        // Inserta de nuevo la lista de clústeres temporales en la lista principal.
        clusters_.insert(clusters_.begin(), tmp_clusters.begin(), tmp_clusters.end());
    }
} // namespace laser_processor 

