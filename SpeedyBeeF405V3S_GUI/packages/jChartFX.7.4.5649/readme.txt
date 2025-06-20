Product: jChartFX
Version: 7.4.5569
Release Date: 06/22/2015
----------------------------

Enhancements:
============

- Tooltip mismatch after Series reordering
- Tooltip offset when html body tag has a style position:relative
- Maps Tooltips data values not changing when hovering from shape to shape
- Interpolation issue when min = max 
- Support CSS class .GaugeMarker
- Crosstab Axes Date Format
- Crosstab RowHeading Datafield sorting
- Crosstab ColumnHeading Partial Dates
- Motifs coloring issues when using different palettes
- New Styles & Motifs

New Features:
============

- Annotation Tooltips
- PictoGraph control. Available as a standalone gauges control and also as a chart extension (PictoGraph and PictoBar)
- Improved Crosstab API and new sort feature (by columns/rows).
- Customizable Color For Inside Labels: Customize (if using CSS styles, check InsideLabel class) the text color for an inside label in non-connected gallery types (e.g.: Bar, Pyramid, Pie, Funnel and Bubble) 
  e.g.: Data point labels text color can be customized to show a lighter/darker color to improve readability.
- Auto-Computed Total value as a Point Label in stacked series. 
- Auto-Computed Total value on Tooltips for stacked series.
- Auto Marker Size: Point marker size is automatically adjusted to avoid collisions. 
- Default Line Alignment to Center, for Stacked Bars and Visible Total value.
- AlternateTag setting in Conditional Attributes, to allow CSS interpolation.
- Fast Line/Curve/Area for when the number of data points exceeds the available pixels on the plot area.
- Tern support: Auto completion and Intelligent JavaScript editing. (http://ternjs.net) 
- TypeScript support: Autocompletion and error highlighting when coding in TypeScript.

- Enhanced Tooltip Features:
	3 different Tooltip Trigger modes:
		Marker
		MarkerThenPlot
		MarkerThenPlot
	Highlighting Markers: Show markers with border for non-visible markers (non-connected gallery types) 
			      Show markers with larger size (connected gallery types) 
	ToolTips with vertical line
	Customizable Tooltips with vertical line settings: width, length, color, X, Y, HorzAlign, VertAlign
	2 different customizable templates: Border Template and Content Template + Smart Defaults 	
	Bar charts shows tooltips on top for both Marker and MarkerThenPlot trigger modes

- Maps Fetaures:
	X/Y to Lat/Long: LocationToPixel and PixelToLocation events
	Annotation ToolTips
	Click Annotation vs Shape event handler
	Bubble Gallery Map Sizing new API: Min/Max/MinPercent

- New Motifs:
	Blinds
	Block	
	Face
	Material
	Metal
	Relief
	Semantic
	Tree	
	Wireframe

Documentation:
=============

The jChartFX documentation is available online and consists of the following:

The jChartFX API reference: Contains descriptions, code samples and tips for each jChart FX API members:

	http://support.softwarefx.com/jChartFX/api

The jChartFX Programmers Guide: Provides detailed articles and samples to help you understand and get the most out of jChartFX.
	
	http://support.softwarefx.com/jChartFX/article/2501235


Plugins:
========

The jChartFX plugins provide additional features like JavaScript/TypeScript code auto completion support on multiple IDE/editors.

Visual Studio Intellisense
--------------------------

jChartFX provides JavaScript IntelliSense auto-completion support if you use Microsoft Visual Studio as your IDE. To enable this feature, include jchartfx.visualstudio.js library in your project and add a reference directive in your JavaScript code as an XML comment:

/// <reference path="Scripts/jchartfx.visualstudio.js"/>
// your JavaScript Code


Tern: Intelligent stand-alone code-analysis editor plugin
---------------------------------------------------------

Tern is intended to be used with as a code editor plugin to enhance the editor's support for intelligent JavaScript editing. 

There is currently Tern support for different editors, like for example:

Emacs
Vim
Sublime Text
Light Table
Eclipse

To enable Autocompletion, argument hints and other editing features, you need to download and install the Tern package (search for instructions on how to install the Tern plugin for your editor) and then include jchartfx.tern.json as an additional JSON type definition in your preferred editor.

For example, when using Sublime Text(2/3), make sure to copy jchartfx.tern.json file in the tern pakage defs folder: 

	\..\Sublime Text 2\Packages\tern_for_sublime\node_modules\tern\defs\jchartfx.tern.json
	

TypeScript code-analysis editor plugin
--------------------------------------

When working with TypeScript code, jChartFX provides auto-completion and error highlighting support around the TypeScript language. You can use the editor/IDE of your choice where TypeScript is supported.

To enable this feature, include jchartfx.d.ts file in your project and add a reference directive in your TypeScript code as an XML comment:

/// <reference path="include/jchartfx.d.ts"/>
// your TypeScript Code


Libraries:
=========

The jChartFX libraries have been split into several .js files to provide granular control over what you add as a reference in your html thus allowing you to limit what the browser downloads when the page is accessed.

The only two files required in any scenario are:
	- jchartfx.coreVector.js or jchartfx.coreBasic.js (depending on whether you want to use Vector based rendering or not)
	- jchartfx.system.js

Beyond those two, you can add or remove references as needed based on the functionality you want to make available on the chart.

Below you will find a complete list of libraries and their description to help you decide whether to reference them or not:


jchartfx.advanced.js
-----------------------
Provides advanced chart features such as Axis Sections, Conditional Attributes and the DataGrid.


jchartfx.animation.js 
-----------------------
Provides animation features. Use this library when enabling animated effects.


jchartfx.annotation.js 
-----------------------
Provides access to the annotation extension. Use this library when you need to overlay drawing objects such as balloons, arrows, floating texts and images.


jchartfx.axistrend.js
-----------------------
Provides access to the AxisTrendg gallery add-on that allows to use the X axis as an easy visualizer of trends or changes displayed in another series.


jchartfx.bullet.js 
-----------------------
Contains the Bullet gallery


jchartFX.coreBasic.js
-----------------------
The lightweight version of the core jChartFX library. Use this library instead of coreVector when a lighter download is favored over more attractive visuals.


jchartFX.coreVector.js
-----------------------
The visually enhanced version of the core jChartFX library. Use this library instead of coreBasic when more complex and attractive visuals are desired.


jchartfx.coreVector3d.js
-----------------------
Adds 3D rendering capabilities to the coreVector library. Requires coreVector.


jchartfx.data.js
-----------------------
Provides advanced data features such as the CrossTab data transform.


jchartfx.density.js
-----------------------
Contains the Density gallery


jchartfx.equalizer.js
-----------------------
Contains the Equalizer bars gallery


jchartfx.funnel.js
-----------------------
Contains the Funnel gallery


jchartfx.gauge.js
-----------------------
Provides access to the Gauge extension. Use this library when you need to display a single variable using gauges. Gauges extension is composed of different types: Radial Gauge, Horizontal Gauge, Vertical Gauge, Trends and Digital Panel. 


jchartfx.handdrawn.js
-----------------------
Provides the HandDrawn extension that allows you to add hand drawn effects to the standard galleries


jchartfx.heatmap.js
-----------------------
Contains the Heatmap gallery


jchartfx.highlow.js
-----------------------
Contains the High-Low gallery


jchartfx.highlowclose.js
-----------------------
Contains the High-Low-Close Financial gallery


jchartfx.maps.js
-----------------------
Provides access to the Maps extension. Use this library when you need to integrate geographic maps with full support of latitude and longitude coordinates


jchartfx.overlaybubble.js
-----------------------
Contains the OverlayBubble gallery


jchartfx.pareto.js
-----------------------
Contains the Pareto Chart gallery


jchartfx.pictograph
-----------------------
Contains the PictoGraph Control, the PictoGraph extension and the PictoBar extension.


jchartfx.pyramid.js
-----------------------
Contains the Pyramid gallery


jchartfx.radar.js
-----------------------
Contains the Radar gallery


jchartfx.rose.js
-----------------------
Contains the Rose gallery


jchartfx.statistical.js
-----------------------
Contains the Statistical extension


jchartfx.sparkline.js
-----------------------
Contains the Sparkline gallery


jchartfx.surface.js
-----------------------
Contains the Surface gallery


jchartfx.system.js (Required)
-----------------------
Contains internal jChartFX classes. This libray is always required.


jchartfx.treemap.js
-----------------------
Contains the Treemap Gallery


jchartfx.ui.js
-----------------------
Provides compatibility with jQuery UI Syntax


jchartfx.userInterface.js
-----------------------
This file contains all the code required by the menu interface, as well as some other user interface elements.
Note: You also need to include the following files to your page to enable jChartFX Menu: jchartfx.userinterface.css

jchartfx.vector.js
-----------------------
This includes the Vector drawing functionality. It is not necessary when using the CoreVector library. It can be used along with the CoreBasic library if you need to use both Vector and non-Vector charts in the same page.


jchartfx.vectorTemplates.js
-----------------------
Provides Vector Gallery Styles.  jChartFX provides a series of templates for most of its chart galleries, with extensive use of gradients


jChartFX Styles and Motifs
================================
jChartFX includes a series of css files that allow you to easily customize any of the controls through style sheets. More than 60 different looks and color styles are available. There is a pair of files for each of the different looks and styles:

- Palette: the jchartfx.palette.<stylename>.js file contains all the color features available.

- Attributes: the jchartfx.attributes.<stylename>.js file contains the rest of the features used in configuration of jChartFX that are not color related, such as fonts, stroke-width, etc.

The reason for this separation is to allow combining any of the styles and motifs with any of the color palettes, which provides more than 600 possible combinations.

In addition to the attributes and palette css files, jChartFX includes a series of .js Motif files, available in the motif subfolder. A motif allows to provide a complete different look to a jChartFX dashboard without the need to code the aesthetics for any of the controls. To use a motif simply include it in your page, after you have included jchartfx.system.js. When you include a motif, you should also include its corresponding attributes and palette css files. For example, if you include jchartfx.motif.hook.js, you should also include jchartfx.attributes.hook.css and jchartfx.palette.hook.css. However, you can use any of the other palette files for a didfferent color combintation, although you you should still use the attribute css file that matches the motif you are using.

The Motifs that are currently included with jChartFX are:
jchartfx.motif.aurora.js
jchartfx.motif.blinds.js
jchartfx.motif.block.js
jchartfx.motif.darkrounded.js
jchartfx.motif.face.js
jchartfx.motif.glow.js
jchartfx.motif.handdrawn.js
jchartfx.motif.healthy.js
jchartfx.motif.hook.js
jchartfx.motif.jchartfx.js
jchartfx.motif.js
jchartfx.motif.lizard.js
jchartfx.motif.material.js
jchartfx.motif.metal.js
jchartfx.motif.metro.js
jchartfx.motif.relief.js
jchartfx.motif.semantic.js
jchartfx.motif.sunken.js
jchartfx.motif.topbar.js
jchartfx.motif.tree.js
jchartfx.motif.vibrant.js
jchartfx.motif.whitespace.js
jchartfx.motif.wireframe.js

You can see an interactive demo of these motifs at www.jchartfx.com/motifs

jChartFX User Interface
=======================
If you want to use the jChartFX UI Menu, you must include the jchartfx.userinterface.css file along with jchartfx.userinterface.js. Please read the online documentation for any additional licensing restriction that may apply to the UI feature.


jChartFX Maps
=============

A complete set of hundreds of maps compatible with the jChartFX Maps extension is available for download at the Software FX Maps Marketplace:

	http://maps.softwarefx.com

Most maps are available in high and low resolutions, so you can select the appropriate size and quality you need depending on your requirements.


Disclaimer:
==========
Information contained in this document is subject to change.

(1) Visual Studio and IntelliSense are registered trademarks of Microsoft Corporation.