.. _coremodules:


The core modules
================

Waf is based on 13 core modules which provide the main functionality of the framework.
They may be used alone, although the support for programming languages or compilers is provided by extensions called Waf tools.

.. graphviz::

   digraph module_deps {
		size="8.0, 12.0";
		"Build" [style="setlinewidth(0.5)",URL="Build.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"ConfigSet" [style="setlinewidth(0.5)",URL="ConfigSet.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Configure" [style="setlinewidth(0.5)",URL="Configure.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Context" [style="setlinewidth(0.5)",URL="Context.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Logs" [style="setlinewidth(0.5)",URL="Logs.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Node" [style="setlinewidth(0.5)",URL="Node.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Options" [style="setlinewidth(0.5)",URL="Options.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Runner" [style="setlinewidth(0.5)",URL="Runner.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Scripting" [style="setlinewidth(0.5)",URL="Scripting.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"TaskGen" [style="setlinewidth(0.5)",URL="TaskGen.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Task" [style="setlinewidth(0.5)",URL="Task.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Utils" [style="setlinewidth(0.5)",URL="Utils.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];
		"Errors" [style="setlinewidth(0.5)",URL="Errors.html",fontname="Vera Sans, DejaVu Sans, Liberation Sans, Arial, Helvetica, sans",height=0.25,shape=box,fontsize=10,fillcolor="#fffea6",style=filled];

		"Build" -> "Runner" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"Build" -> "TaskGen" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"Build" -> "ConfigSet" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"Build" -> "Options" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"ConfigSet" -> "Utils" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"ConfigSet" -> "Logs" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Configure" -> "Build" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Context" -> "Logs" [arrowsize=0.5,style="setlinewidth(0.5)"];
		"Context" -> "Node" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Node" -> "Utils" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Options" -> "Context" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Runner" -> "Task" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Scripting" -> "Configure" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"TaskGen" -> "Task" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Task" -> "Logs" [arrowsize=0.5,style="setlinewidth(0.5)"];

		"Utils" -> "Errors" [arrowsize=0.5,style="setlinewidth(0.5)"];
   }

