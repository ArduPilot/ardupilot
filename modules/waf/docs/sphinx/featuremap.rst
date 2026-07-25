.. _featuremap:

Feature reference
=================

.. include:: featuremap_example.txt
Feature \*
==========

.. graphviz::

	digraph feature_all {
		size="8.0, 12.0";
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"grep_for_endianness_fun" [style="setlinewidth(0.5)",URL="../tools/c_tests.html#waflib.Tools.c_tests.grep_for_endianness_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"link_lib_test_fun" [style="setlinewidth(0.5)",URL="../tools/c_tests.html#waflib.Tools.c_tests.link_lib_test_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_install_task" [style="setlinewidth(0.5)",URL="../Build.html#waflib.Build.process_install_task",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_javadoc" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.create_javadoc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_subst" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_subst",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_marshal" [style="setlinewidth(0.5)",URL="../tools/glib2.html#waflib.Tools.glib2.process_marshal",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_java" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.apply_java",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"jar_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.jar_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_tex" [style="setlinewidth(0.5)",URL="../tools/tex.html#waflib.Tools.tex.apply_tex",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_intltool_in_f" [style="setlinewidth(0.5)",URL="../tools/intltool.html#waflib.Tools.intltool.apply_intltool_in_f",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"feature_py" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.feature_py",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_mocs" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.process_mocs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_rule" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_rule",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_enums" [style="setlinewidth(0.5)",URL="../tools/glib2.html#waflib.Tools.glib2.process_enums",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_objs" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_objs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"link_main_routines_tg_method" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.link_main_routines_tg_method",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_cs" [style="setlinewidth(0.5)",URL="../tools/cs.html#waflib.Tools.cs.apply_cs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_javadoc" -> "process_rule" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_rule" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_subst" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_install_task" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "link_lib_test_fun" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_objs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "link_main_routines_tg_method" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_tex" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "jar_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_marshal" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_enums" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_mocs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "feature_py" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_intltool_in_f" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_cs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"grep_for_endianness_fun" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_rule" -> "process_subst" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_rule" -> "process_install_task" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature asm
===========

.. graphviz::

	digraph feature_asm {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_flags_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_nasm_vars" [style="setlinewidth(0.5)",URL="../tools/nasm.html#waflib.Tools.nasm.apply_nasm_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_msvc" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_flags_msvc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_msvc" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature c
=========

.. graphviz::

	digraph feature_c {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_flags_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_macosx_deployment_target" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.set_macosx_deployment_target",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_msvc" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_flags_msvc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_msvc" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cprogram
================

.. graphviz::

	digraph feature_cprogram {
		size="8.0, 12.0";
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cs
==========

.. graphviz::

	digraph feature_cs {
		size="8.0, 12.0";
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"debug_cs" [style="setlinewidth(0.5)",URL="../tools/cs.html#waflib.Tools.cs.debug_cs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"use_cs" [style="setlinewidth(0.5)",URL="../tools/cs.html#waflib.Tools.cs.use_cs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_cs" [style="setlinewidth(0.5)",URL="../tools/cs.html#waflib.Tools.cs.apply_cs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"use_cs" -> "apply_cs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"debug_cs" -> "apply_cs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"debug_cs" -> "use_cs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_cs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cshlib
==============

.. graphviz::

	digraph feature_cshlib {
		size="8.0, 12.0";
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_bundle" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"init_pyext" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cxx
===========

.. graphviz::

	digraph feature_cxx {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_flags_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_macosx_deployment_target" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.set_macosx_deployment_target",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_msvc" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_flags_msvc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_msvc" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cxxprogram
==================

.. graphviz::

	digraph feature_cxxprogram {
		size="8.0, 12.0";
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature cxxshlib
================

.. graphviz::

	digraph feature_cxxshlib {
		size="8.0, 12.0";
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_bundle" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"init_pyext" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature d
=========

.. graphviz::

	digraph feature_d {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_flags_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_header" [style="setlinewidth(0.5)",URL="../tools/d.html#waflib.Tools.d.process_header",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_msvc" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_flags_msvc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_msvc" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature dshlib
==============

.. graphviz::

	digraph feature_dshlib {
		size="8.0, 12.0";
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature fake_lib
================

.. graphviz::

	digraph feature_fake_lib {
		size="8.0, 12.0";
		"process_lib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_lib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
	}



Feature fake_obj
================

.. graphviz::

	digraph feature_fake_obj {
		size="8.0, 12.0";
		"process_objs" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_objs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "process_objs" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature fc
==========

.. graphviz::

	digraph feature_fc {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_flags_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macplist" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macplist",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_manifest",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_task_macapp" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.create_task_macapp",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_flags_msvc" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.apply_flags_msvc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macplist" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"create_task_macapp" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_flags_msvc" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature fcprogram
=================

.. graphviz::

	digraph feature_fcprogram {
		size="8.0, 12.0";
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature fcprogram_test
======================

.. graphviz::

	digraph feature_fcprogram_test {
		size="8.0, 12.0";
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature fcshlib
===============

.. graphviz::

	digraph feature_fcshlib {
		size="8.0, 12.0";
		"apply_implib" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_implib",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_manifest_ifort" [style="setlinewidth(0.5)",URL="../tools/ifort.html#waflib.Tools.ifort.apply_manifest_ifort",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_manifest_ifort" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_implib" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature glib2
=============

.. graphviz::

	digraph feature_glib2 {
		size="8.0, 12.0";
		"process_settings" [style="setlinewidth(0.5)",URL="../tools/glib2.html#waflib.Tools.glib2.process_settings",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
	}



Feature grep_for_endianness
===========================

.. graphviz::

	digraph feature_grep_for_endianness {
		size="8.0, 12.0";
		"grep_for_endianness_fun" [style="setlinewidth(0.5)",URL="../tools/c_tests.html#waflib.Tools.c_tests.grep_for_endianness_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"grep_for_endianness_fun" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature gresource
=================

.. graphviz::

	digraph feature_gresource {
		size="8.0, 12.0";
		"process_gresource_bundle" [style="setlinewidth(0.5)",URL="../tools/glib2.html#waflib.Tools.glib2.process_gresource_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
	}



Feature includes
================

.. graphviz::

	digraph feature_includes {
		size="8.0, 12.0";
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature install_task
====================

.. graphviz::

	digraph feature_install_task {
		size="8.0, 12.0";
		"process_rule" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_rule",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_install_task" [style="setlinewidth(0.5)",URL="../Build.html#waflib.Build.process_install_task",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "process_install_task" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_rule" -> "process_install_task" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature intltool_in
===================

.. graphviz::

	digraph feature_intltool_in {
		size="8.0, 12.0";
		"apply_intltool_in_f" [style="setlinewidth(0.5)",URL="../tools/intltool.html#waflib.Tools.intltool.apply_intltool_in_f",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "apply_intltool_in_f" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature intltool_po
===================

.. graphviz::

	digraph feature_intltool_po {
		size="8.0, 12.0";
		"apply_intltool_po" [style="setlinewidth(0.5)",URL="../tools/intltool.html#waflib.Tools.intltool.apply_intltool_po",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
	}



Feature jar
===========

.. graphviz::

	digraph feature_jar {
		size="8.0, 12.0";
		"apply_java" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.apply_java",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"jar_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.jar_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"use_javac_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.use_javac_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"use_jar_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.use_jar_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"use_jar_files" -> "jar_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "jar_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"jar_files" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"jar_files" -> "use_javac_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature javac
=============

.. graphviz::

	digraph feature_javac {
		size="8.0, 12.0";
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_java" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.apply_java",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"jar_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.jar_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"use_javac_files" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.use_javac_files",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"use_javac_files" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"jar_files" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"jar_files" -> "use_javac_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "apply_java" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "use_javac_files" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature javadoc
===============

.. graphviz::

	digraph feature_javadoc {
		size="8.0, 12.0";
		"process_rule" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_rule",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"create_javadoc" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.create_javadoc",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"create_javadoc" -> "process_rule" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature link_lib_test
=====================

.. graphviz::

	digraph feature_link_lib_test {
		size="8.0, 12.0";
		"link_lib_test_fun" [style="setlinewidth(0.5)",URL="../tools/c_tests.html#waflib.Tools.c_tests.link_lib_test_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "link_lib_test_fun" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature link_main_routines_func
===============================

.. graphviz::

	digraph feature_link_main_routines_func {
		size="8.0, 12.0";
		"link_main_routines_tg_method" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.link_main_routines_tg_method",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "link_main_routines_tg_method" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature perlext
===============

.. graphviz::

	digraph feature_perlext {
		size="8.0, 12.0";
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature py
==========

.. graphviz::

	digraph feature_py {
		size="8.0, 12.0";
		"feature_py" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.feature_py",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "feature_py" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature pyembed
===============

.. graphviz::

	digraph feature_pyembed {
		size="8.0, 12.0";
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature pyext
=============

.. graphviz::

	digraph feature_pyext {
		size="8.0, 12.0";
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_bundle" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.set_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "set_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"init_pyext" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature qt5
===========

.. graphviz::

	digraph feature_qt5 {
		size="8.0, 12.0";
		"apply_qt5" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.apply_qt5",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_mocs" [style="setlinewidth(0.5)",URL="../tools/qt5.html#waflib.Tools.qt5.process_mocs",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_qt5" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "process_mocs" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature rubyext
===============

.. graphviz::

	digraph feature_rubyext {
		size="8.0, 12.0";
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_rubyext" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.init_rubyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_bundle" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_source" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "init_rubyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_link" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature seq
===========

.. graphviz::

	digraph feature_seq {
		size="8.0, 12.0";
		"sequence_order" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.sequence_order",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
	}



Feature subst
=============

.. graphviz::

	digraph feature_subst {
		size="8.0, 12.0";
		"process_rule" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_rule",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_subst" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_subst",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" -> "process_subst" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_rule" -> "process_subst" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature test
============

.. graphviz::

	digraph feature_test {
		size="8.0, 12.0";
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_test" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature test_exec
=================

.. graphviz::

	digraph feature_test_exec {
		size="8.0, 12.0";
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"test_exec_fun" [style="setlinewidth(0.5)",URL="../tools/c_config.html#waflib.Tools.c_config.test_exec_fun",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"test_exec_fun" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature tex
===========

.. graphviz::

	digraph feature_tex {
		size="8.0, 12.0";
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_tex" [style="setlinewidth(0.5)",URL="../tools/tex.html#waflib.Tools.tex.apply_tex",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_source" -> "apply_tex" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature use
===========

.. graphviz::

	digraph feature_use {
		size="8.0, 12.0";
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"make_test" [style="setlinewidth(0.5)",URL="../tools/waf_unit_test.html#waflib.Tools.waf_unit_test.make_test",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_source" [style="setlinewidth(0.5)",URL="../TaskGen.html#waflib.TaskGen.process_source",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_full_paths_hpux" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.set_full_paths_hpux",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_test" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_incpaths" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"process_use" -> "process_source" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_full_paths_hpux" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature uselib
==============

.. graphviz::

	digraph feature_uselib {
		size="8.0, 12.0";
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_perlext" [style="setlinewidth(0.5)",URL="../tools/perl.html#waflib.Tools.perl.init_perlext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_ruby_so_name" [style="setlinewidth(0.5)",URL="../tools/ruby.html#waflib.Tools.ruby.apply_ruby_so_name",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_incpaths",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyembed" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyembed",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"init_pyext" [style="setlinewidth(0.5)",URL="../tools/python.html#waflib.Tools.python.init_pyext",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_lib_pat" [style="setlinewidth(0.5)",URL="../tools/fc_config.html#waflib.Tools.fc_config.set_lib_pat",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"set_classpath" [style="setlinewidth(0.5)",URL="../tools/javaw.html#waflib.Tools.javaw.set_classpath",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_bundle" [style="setlinewidth(0.5)",URL="../tools/c_osx.html#waflib.Tools.c_osx.apply_bundle",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_incpaths" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"set_classpath" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_bundle" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "set_lib_pat" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "apply_ruby_so_name" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_perlext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyext" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"propagate_uselib_vars" -> "init_pyembed" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature vnum
============

.. graphviz::

	digraph feature_vnum {
		size="8.0, 12.0";
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_vnum",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"apply_link" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.apply_link",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"apply_vnum" -> "apply_link" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"apply_vnum" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature winapp
==============

.. graphviz::

	digraph feature_winapp {
		size="8.0, 12.0";
		"make_windows_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_windows_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_windows_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_windows_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}



Feature winphoneapp
===================

.. graphviz::

	digraph feature_winphoneapp {
		size="8.0, 12.0";
		"process_use" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.process_use",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"propagate_uselib_vars" [style="setlinewidth(0.5)",URL="../tools/ccroot.html#waflib.Tools.ccroot.propagate_uselib_vars",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10];
		"make_winphone_app" [style="setlinewidth(0.5)",URL="../tools/msvc.html#waflib.Tools.msvc.make_winphone_app",target="_top",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape="rectangle",fontsize=10,fillcolor="#fffea6",style=filled];
		"make_winphone_app" -> "propagate_uselib_vars" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"make_winphone_app" -> "process_use" [arrowsize=0.5,style="setlinewidth(0.5)"];
	}


