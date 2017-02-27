#!/usr/bin/env python
'''
    This file is a part of TachibanaSite.

    File:   utils/template.py
    Author: Chris McKinney
    Edited: Aug 10 2016
    Editor: Chris McKinney

    Description:

    Utilities for working with template files.

    Edit History:

    0.8.10  - Added module support.

    License:

    Copyright 2016 Chris McKinney

    Licensed under the Apache License, Version 2.0 (the "License"); you may not
    use this file except in compliance with the License.  You may obtain a copy
    of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

# Render the template in the given environment.
def render_template_env(filename, **environment):
    from bottle import SimpleTemplate
    with open(filename) as f:
        return SimpleTemplate(f, name=filename).render(**environment)

# Render the template in the default environment, with additional values.
def render_template(filename, **environment):
    env = DEFAULT_TEMPLATE_ENV.copy()
    env.update(environment)
    return render_template_env(filename, **env)

def _get_tpl_lib_bindings():
    import os, sys, importlib
    from os.path import dirname, realpath
    installPath = dirname(dirname(realpath(__file__)))
    modulesPath = os.path.join(installPath, 'modules')
    sys.path.append(modulesPath)
    moduleNames = os.listdir(modulesPath)
    moduleNames.sort()
    bindings = []
    for name in moduleNames:
        if '.' in name:
            continue
        moduleInit = os.path.join(os.path.join(modulesPath, name),
                '__init__.py')
        if not os.path.isfile(moduleInit):
            continue
        try:
            module = importlib.import_module(name)
            bindings.append(module.TACHIBANASITE_TPL_LIB_BINDINGS)
        except ImportError:
            continue
        except AttributeError:
            continue
    return bindings

TEMPLATE_LIB_BINDINGS = []#_get_tpl_lib_bindings()

# The default environment for render_template.
DEFAULT_TEMPLATE_ENV = {
        '_GET': {},
        'render_template_env': render_template_env,
        'render_template': render_template,
        }

for bindings in TEMPLATE_LIB_BINDINGS:
    DEFAULT_TEMPLATE_ENV.update(bindings)

# Render each template file provided to the script.
def main():
    import sys, json
    _GET = json.loads(sys.argv[1])
    for filename in sys.argv[2:]:
        print(render_template(filename, _GET=_GET))

if __name__ == '__main__':
    import sys
    reload(sys)
    sys.setdefaultencoding('utf-8')
    main()
