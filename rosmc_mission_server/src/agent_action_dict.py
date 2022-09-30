#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The BSD 3-Clause License
#
# Copyright (c) 2022, DLR-RM All rights reserved.
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, 
#    this list of conditions and the following disclaimer in the documentation 
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors 
#    may be used to endorse or promote products derived from this software 
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Contributors:
# Ryo Sakagami <ryo.sakagami@dlr.de>

from functools import reduce  # forward compatibility for Python 3
import json
import operator
import os
import pprint
import yaml


def read_skill_from_file(core_data_json_path):
    name = ''
    inputs = {}

    with open(core_data_json_path) as json_file:
        data = json.load(json_file)

        # If input_data_ports does not exist at all, abort; something is wrong
        if 'input_data_ports' not in data:
            print('ERROR: no input_data_ports given for this statemachine')
            return None
        # If name does not exist at all, abort; something is wrong
        if 'name' not in data:
            print('ERROR: no name given for this statemachine')
            return None

        name = str(data['name'])
        input_data_ports = data['input_data_ports']

        # If length of input_data_ports is 0, no input is given; input is string 'None'
        if len(input_data_ports) == 0:
            inputs = 'None'

        # Read inputs
        for key in input_data_ports:
            val = input_data_ports[key]
            input_name = str(val['name'])
            data_type = str(val['data_type']['__type__'].split('.')[-1])
            default_value = val['default_value']
            inputs[input_name] = {
                'data_type': data_type,
                'default_value': default_value
            }

    return name, inputs


def get_root_state_storage_id(statemachine_json_path):
    root_state_storage_id = None
    with open(statemachine_json_path) as json_file:
        data = json.load(json_file)
        root_state_storage_id = data['root_state_storage_id']
    return root_state_storage_id


def read_all_skills(skill_root_dir):
    skills = {}
    for dirpath, dirs, files in os.walk(skill_root_dir):
        # do not process .git dir
        if '.git' in dirs or '.git' in dirpath:
            continue
        # find dir having statemachine.json
        if 'statemachine.json' not in files:
            continue
        root_state_storage_id = get_root_state_storage_id(os.path.join(dirpath, 'statemachine.json'))
        core_data_json_path = os.path.join(dirpath, root_state_storage_id, 'core_data.json')
        name, inputs = read_skill_from_file(core_data_json_path)
        print(name)
        skills[name] = inputs
    return skills


def yaml2dict(yaml_path):
    with open(yaml_path, 'r') as stream:
        try:
            d = yaml.safe_load(stream)
            return d
        except yaml.YAMLError as exc:
            print(exc)
            return None


def parse_agent_actionlib_config(agent_actionlib_config_path):
    d = yaml2dict(agent_actionlib_config_path)
    if d is None:
        return None
    # resolve environment variables
    for key in d:
        d[key]['skill_root_dir'] = os.path.expandvars(d[key]['skill_root_dir'])
    return d


def agent_action_dict(agent_actionlib_config):
    if agent_actionlib_config is None:
        return None
    d = {}
    for agent_name in agent_actionlib_config:
        skill_root_dir = agent_actionlib_config[agent_name]['skill_root_dir']
        #name = agent_acitonlib_config[agent_name]['name']
        d[agent_name] = read_all_skills(skill_root_dir)
    return d

