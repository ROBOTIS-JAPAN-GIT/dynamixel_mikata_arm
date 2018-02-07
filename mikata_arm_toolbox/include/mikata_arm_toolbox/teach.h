/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef __TEACH_H
#define __TEACH_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "parameters.h"

/** Class definition for teaching sample **/

struct node {
  std::string name;
  std::vector<int> id_vec;
  std::vector<double> q;
};

class Teach {

private:
  std::vector<node> Nodes;

public:
  int getSize() { return Nodes.size(); }
  node* getNode(int index) { return &Nodes[index-1]; }
  node* getNode(std::string name);
  void addNode(std::string _name, std::vector<int> _id_vec, std::vector<double> _q);
  bool deleteNode(int index);
  void print();
  void clearAll() { Nodes.erase(Nodes.begin(), Nodes.end()); };
};

node* Teach::getNode(std::string name) {
  for (int i=0; i<Nodes.size(); i++)
    if(Nodes[i].name == name)
      return &Nodes[i];
  return NULL;
}

void Teach::addNode(std::string _name, std::vector<int> _id_vec, std::vector<double> _q) {
  node new_node;
  new_node.name = _name;
  new_node.id_vec = _id_vec;
  new_node.q = _q;

  Nodes.push_back(new_node);
}
bool Teach::deleteNode(int index) {
  if (index > Nodes.size())
    return false;
  else {
    Nodes.erase(Nodes.begin() + index-1);
    return true;
  }
}
void Teach::print() {
  for (unsigned i=0; i<Nodes.size(); ++i) 
    std::cout << std::setw(2) << i+1 << ". " << Nodes[i].name << std::endl;
}


/** Read and write from file **/

template<typename Func> void read_file(std::string fileName, Func func) {
  std::ifstream infile;
  char data[100];

  infile.open(fileName.c_str(), std::ios::in);
  
  while (infile.getline(data,100)) {
    std::istringstream iss(data);
    std::string name, buffer;
    std::vector<int> id_vec;
    std::vector<double> q;
    int SPACE_KEY = 32;
    int id=0;
    
    getline(iss, name, ',');
    boost::trim(name);
    if (!name.size() || name[0] == '#') continue;
    while (getline(iss, buffer, ',')) {
      id++;
      try {
	q.push_back(std::stod(buffer));
	id_vec.push_back(id);
      } catch (std::invalid_argument) { continue; }
    }
    func(name, id_vec, q);
  }
  infile.close();
}

void write_file (std::string fileName, Teach teaching) {
  std::ofstream outfile;
  outfile.open(fileName.c_str(), std::ios::out);
  for(int i=1; i<=teaching.getSize(); i++) {
    node* node = teaching.getNode(i);
    outfile << node->name;
    for(int k=1; k<=LINK_NUM_GRIPPER; k++) {
      outfile << " ,";
      std::vector<int>::iterator it = std::find(node->id_vec.begin(), node->id_vec.end(), k);
      if (it != node->id_vec.end())
	outfile << node->q[std::distance(node->id_vec.begin(), it)];
    }
    outfile << std::endl;
  }
  outfile.close();
}


/** Declare global variable **/

Teach teaching_data;

#endif
