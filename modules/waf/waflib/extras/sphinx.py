"""Support for Sphinx documentation

This is a wrapper for sphinx-build program. Please note that sphinx-build supports only
one output format at a time, but the tool can create multiple tasks to handle more.
The output formats can be passed via the sphinx_output_format, which is an array of
strings. For backwards compatibility if only one output is needed, it can be passed
as a single string.
The default output format is html.

Specific formats can be installed in different directories by specifying the
install_path_<FORMAT> attribute. If not defined, the standard install_path
will be used instead.

Example wscript:

def configure(cnf):
    conf.load('sphinx')

def build(bld):
    bld(
        features='sphinx',
        sphinx_source='sources',  # path to source directory
        sphinx_options='-a -v',  # sphinx-build program additional options
        sphinx_output_format=['html', 'man'],  # output format of sphinx documentation
        install_path_man='${DOCDIR}/man'       # put man pages in a specific directory
        )

"""

from waflib.Node import Node
from waflib import Utils
from waflib import Task
from waflib.TaskGen import feature, after_method


def configure(cnf):
    """Check if sphinx-build program is available and loads gnu_dirs tool."""
    cnf.find_program('sphinx-build', var='SPHINX_BUILD', mandatory=False)
    cnf.load('gnu_dirs')


@feature('sphinx')
def build_sphinx(self):
    """Builds sphinx sources.
    """
    if not self.env.SPHINX_BUILD:
        self.bld.fatal('Program SPHINX_BUILD not defined.')
    if not getattr(self, 'sphinx_source', None):
        self.bld.fatal('Attribute sphinx_source not defined.')
    if not isinstance(self.sphinx_source, Node):
        self.sphinx_source = self.path.find_node(self.sphinx_source)
    if not self.sphinx_source:
        self.bld.fatal('Can\'t find sphinx_source: %r' % self.sphinx_source)

    # In the taskgen we have the complete list of formats
    Utils.def_attrs(self, sphinx_output_format='html')
    self.sphinx_output_format = Utils.to_list(self.sphinx_output_format)

    self.env.SPHINX_OPTIONS = getattr(self, 'sphinx_options', [])

    for source_file in self.sphinx_source.ant_glob('**/*'):
        self.bld.add_manual_dependency(self.sphinx_source, source_file)

    for cfmt in self.sphinx_output_format:
        sphinx_build_task = self.create_task('SphinxBuildingTask')
        sphinx_build_task.set_inputs(self.sphinx_source)
        # In task we keep the specific format this task is generating
        sphinx_build_task.env.SPHINX_OUTPUT_FORMAT = cfmt

        # the sphinx-build results are in <build + output_format> directory
        sphinx_build_task.sphinx_output_directory = self.path.get_bld().make_node(cfmt)
        sphinx_build_task.set_outputs(sphinx_build_task.sphinx_output_directory)
        sphinx_build_task.sphinx_output_directory.mkdir()

        Utils.def_attrs(sphinx_build_task, install_path=getattr(self, 'install_path_' + cfmt, getattr(self, 'install_path', get_install_path(sphinx_build_task))))


def get_install_path(object):
    if object.env.SPHINX_OUTPUT_FORMAT == 'man':
        return object.env.MANDIR
    elif object.env.SPHINX_OUTPUT_FORMAT == 'info':
        return object.env.INFODIR
    else:
        return object.env.DOCDIR


class SphinxBuildingTask(Task.Task):
    color = 'BOLD'
    run_str = '${SPHINX_BUILD} -M ${SPHINX_OUTPUT_FORMAT} ${SRC} ${TGT} -d ${TGT[0].bld_dir()}/doctrees-${SPHINX_OUTPUT_FORMAT} ${SPHINX_OPTIONS}'

    def keyword(self):
        return 'Compiling (%s)' % self.env.SPHINX_OUTPUT_FORMAT

    def runnable_status(self):

        for x in self.run_after:
            if not x.hasrun:
                return Task.ASK_LATER

        self.signature()
        ret = Task.Task.runnable_status(self)
        if ret == Task.SKIP_ME:
            # in case the files were removed
            self.add_install()
        return ret


    def post_run(self):
        self.add_install()
        return Task.Task.post_run(self)


    def add_install(self):
        nodes = self.sphinx_output_directory.ant_glob('**/*', quiet=True)
        self.outputs += nodes
        self.generator.add_install_files(install_to=self.install_path,
                                         install_from=nodes,
                                         postpone=False,
                                         cwd=self.sphinx_output_directory.make_node(self.env.SPHINX_OUTPUT_FORMAT),
                                         relative_trick=True)
