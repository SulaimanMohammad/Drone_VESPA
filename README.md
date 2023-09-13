# Drone_VESPA

## Setup Raspberry pi
- Connect aspberry pi to pixhawk using Telemetry 2 port
- Configure raspberry pi to be able to communicate with pixhawk 

```bash
        git clone https://github.com/SulaimanMohammad/Drone_VESPA.git
        cd Drone_VESPA
``` 
```bash
        ./rpi_setup.sh
```
- Check for updates, create logs directory, set permissions 

```bash
        ./update_repo.sh
``` 

## Scripts Map 
- "drone_ardupilot.py": API to move take off, and move the drone 
- "unit_tests" contains implementation of drone_ardupilot 
- "expan.py", "spaning.py" are the pahses in VESPA Algoithm 
    To see more about this algorithm check the [simulation here](https://github.com/SulaimanMohammad/self-organized-uav)


## Run tests 
- All tests in unit_tests can be used to commuinicate with raspberry pi, telemetry and simulation 
- The test used in the algorithm of VESPA is  body_frame_move.py 
    - It uses the movement using Yaw , distance and PID 
    
    ```bash
        python3 body_frame_move.py
    ``` 

```plantuml
@startuml
<mxfile host="app.diagrams.net" modified="2023-09-11T12:57:48.876Z" agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36" etag="O1slZWHyWAMAiTp3cavT" version="21.7.1" type="device">
  <diagram id="2o8Grx6nBrJDEr8GREQO" name="Page-1">
    <mxGraphModel dx="3317" dy="2207" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="850" pageHeight="1100" math="0" shadow="0">
      <root>
        <mxCell id="0" />
        <mxCell id="1" parent="0" />
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-35" value="" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="-1180" y="790" width="1160" height="500" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-12" value="" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="-1180" y="-215" width="1170" height="670" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-1" value="" style="rounded=0;whiteSpace=wrap;html=1;shape=mxgraph.dfd.loop" parent="1" vertex="1">
          <mxGeometry x="-1150" y="820" width="1130" height="240" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-376" value="" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="10" y="-210" width="2420" height="1500" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-257" value="" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="950" y="19" width="400" height="170" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-18" value="" style="html=1;dashed=0;whiteSpace=wrap;shape=mxgraph.dfd.loop" parent="1" vertex="1">
          <mxGeometry x="650" y="444.5" width="570" height="195.5" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-4" value="&lt;font style=&quot;font-size: 14px;&quot;&gt;Lunch Listening thread&amp;nbsp;&lt;/font&gt;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="173" y="563" width="160" height="42" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-367" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-5" target="-KWZgO-KfmSXsMUV4DuB-31" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-5" value="&lt;font style=&quot;font-size: 14px;&quot;&gt;Check the state&amp;nbsp;&lt;/font&gt;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="340" y="629.5" width="132.5" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-27" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;exitX=0.5;exitY=0;exitDx=0;exitDy=0;entryX=0.5;entryY=1;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-9" target="-KWZgO-KfmSXsMUV4DuB-26" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-59" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=-0.007;entryY=0.546;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-9" target="-KWZgO-KfmSXsMUV4DuB-58" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-9" value="State is &lt;br&gt;&lt;b&gt;Free or Border&lt;/b&gt;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="664" y="728.5" width="110" height="110" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-17" value="" style="html=1;dashed=0;whiteSpace=wrap;shape=mxgraph.dfd.loop" parent="1" vertex="1">
          <mxGeometry x="-1170" y="-160" width="1160" height="600" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-302" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-19" edge="1">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="970" y="100" as="targetPoint" />
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-30" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-20" target="-KWZgO-KfmSXsMUV4DuB-26" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="710" y="160" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-29" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-21" target="-KWZgO-KfmSXsMUV4DuB-26" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="640" y="242" />
              <mxPoint x="640" y="525" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-24" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="724" y="698.5" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-63" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-26" target="-KWZgO-KfmSXsMUV4DuB-39" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-26" value="" style="shape=hexagon;perimeter=hexagonPerimeter2;whiteSpace=wrap;html=1;size=0.25" parent="1" vertex="1">
          <mxGeometry x="664" y="500" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-25" value="check flages" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="669" y="509" width="90" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-33" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-31" target="-KWZgO-KfmSXsMUV4DuB-9" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-9" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.087;entryY=-0.003;entryDx=0;entryDy=0;entryPerimeter=0;" edge="1" parent="1" source="-KWZgO-KfmSXsMUV4DuB-31" target="PSZtOdJVlawKTp13Huel-31">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-31" value="Drone is &lt;b&gt;Sink&lt;/b&gt;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="500" y="594.5" width="110" height="110" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-34" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="600" y="620" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-36" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="512" y="698.5" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-52" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-37" target="-KWZgO-KfmSXsMUV4DuB-51" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-225" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-37" target="-KWZgO-KfmSXsMUV4DuB-221" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-37" value="&amp;nbsp; &lt;br&gt;listener_current_&lt;br&gt;updated_irremovable&lt;br&gt;raised" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="940" y="469" width="130" height="110" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-41" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-39" target="-KWZgO-KfmSXsMUV4DuB-37" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-45" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.459;entryY=0.012;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-39" target="-KWZgO-KfmSXsMUV4DuB-44" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-39" value="one is&lt;br&gt;&amp;nbsp;raised" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="799" y="485" width="80" height="80" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-42" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="870" y="500" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-43" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="840" y="550" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-46" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.625;entryY=1;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-44" target="-KWZgO-KfmSXsMUV4DuB-26" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-44" value="sleep 0.1 s" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="797" y="592" width="90" height="35" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-61" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-51" target="-KWZgO-KfmSXsMUV4DuB-58" edge="1">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="940" y="770" as="targetPoint" />
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-51" value="Clear the flag&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="960" y="610" width="90" height="20" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-53" value="YES&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1010" y="575" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-54" value="&amp;nbsp;The state became &lt;br&gt;Irremovable" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1040" y="600" width="120" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-55" value="NO&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1080" y="490" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-57" value="NO&lt;br&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="774" y="755" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-60" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;state is irremovable&amp;nbsp;&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="764" y="785" width="140" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-222" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-166" target="-KWZgO-KfmSXsMUV4DuB-221" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1720" y="731" />
              <mxPoint x="1720" y="783" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-223" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.334;entryY=0.83;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-171" target="-KWZgO-KfmSXsMUV4DuB-221" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-228" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-221" target="-KWZgO-KfmSXsMUV4DuB-227" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-315" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-221" target="-KWZgO-KfmSXsMUV4DuB-314" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-221" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;&amp;nbsp;&amp;nbsp;&lt;br&gt;listener_end_of_spanning&lt;br&gt;raised&lt;/font&gt;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1740" y="706" width="170" height="155" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-230" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-227" target="-KWZgO-KfmSXsMUV4DuB-229" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-227" value="Clear the flag&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1950" y="758.5" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-229" value="&lt;font style=&quot;font-size: 19px;&quot;&gt;END&lt;/font&gt;" style="html=1;dashed=0;whiteSpace=wrap;shape=mxgraph.dfd.start" parent="1" vertex="1">
          <mxGeometry x="2320" y="760.88" width="110" height="45.25" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-285" value="" style="group" parent="1" vertex="1" connectable="0">
          <mxGeometry x="1510" y="-140" width="900" height="360" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-245" value="" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="487.5" y="267" width="45" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-102" value="" style="rhombus;whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="554" y="140" width="60" height="60" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-66" value="" style="whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry width="900" height="360" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-68" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;find_close_neigboor_2sink&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry width="180" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-69" value="Extract the 6&amp;nbsp;neighbors&lt;br&gt;NO need for S0 ( current position)" style="rounded=0;whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="20" y="40" width="160" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-70" value="Sort the list by distance&amp;nbsp;&lt;br&gt;&lt;b&gt;Ascending&lt;/b&gt;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="210" y="40" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-71" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-69" target="-KWZgO-KfmSXsMUV4DuB-70" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-72" value="slice the first 3&amp;nbsp;&lt;br&gt;( 3 spots close to sink)&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="340" y="40" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-73" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-70" target="-KWZgO-KfmSXsMUV4DuB-72" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-74" value="Filter the list of the spot to keep only the occupied&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="470" y="40" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-75" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-72" target="-KWZgO-KfmSXsMUV4DuB-74" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-76" value="There are &lt;br&gt;drones&amp;nbsp;&lt;br&gt;close to &lt;br&gt;sink&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="35" y="120" width="100" height="100" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-79" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-74" target="-KWZgO-KfmSXsMUV4DuB-76" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="520" y="110" />
              <mxPoint x="20" y="110" />
              <mxPoint x="20" y="170" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-80" value="Search in the list of neighbors&lt;br&gt;to drone was previously a border&amp;nbsp;&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="330" y="145" width="200" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-82" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="120" y="130" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-83" value="Find many&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="552" y="137.5" width="70" height="65" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-84" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-80" target="-KWZgO-KfmSXsMUV4DuB-83" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-85" value="Find the closest to sink&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="672" y="145" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-86" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-83" target="-KWZgO-KfmSXsMUV4DuB-85" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-87" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="617" y="137.5" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-88" value="Retunr ID&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="792" y="145" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-90" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=1;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-83" target="-KWZgO-KfmSXsMUV4DuB-88" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="587" y="230" />
              <mxPoint x="842" y="230" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-89" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-85" target="-KWZgO-KfmSXsMUV4DuB-88" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-91" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="582" y="195" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-92" value="Only one&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="772" y="200" width="70" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-104" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="100" y="207" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-242" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=1;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-183" target="-KWZgO-KfmSXsMUV4DuB-83" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="570" y="272" />
              <mxPoint x="570" y="207" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-183" value="Exist &lt;br&gt;Irremovable" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="460" y="227" width="100" height="90" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-186" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="530" y="230" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-187" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="540" y="297" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-189" value="Retunr -1&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="792" y="267" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-234" value="" style="shape=hexagon;perimeter=hexagonPerimeter2;whiteSpace=wrap;html=1;size=0.25" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="35" y="247" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-235" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-76" target="-KWZgO-KfmSXsMUV4DuB-234" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-233" value="chek flage" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="45" y="257" width="80" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-251" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.258;entryY=0.833;entryDx=0;entryDy=0;entryPerimeter=0;exitX=0.5;exitY=1;exitDx=0;exitDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-236" target="-KWZgO-KfmSXsMUV4DuB-183" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="242" y="350" />
              <mxPoint x="486" y="350" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-236" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;&amp;nbsp;&amp;nbsp;&lt;br&gt;listener_neighbor_&lt;br&gt;update_state&lt;br&gt;raised&lt;/font&gt;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="172" y="209.5" width="140" height="125" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-241" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.033;entryY=0.505;entryDx=0;entryDy=0;entryPerimeter=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-234" target="-KWZgO-KfmSXsMUV4DuB-236" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-240" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-238" target="-KWZgO-KfmSXsMUV4DuB-183" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-238" value="wait until mutex is unlocked" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="340" y="247" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-239" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-236" target="-KWZgO-KfmSXsMUV4DuB-238" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-246" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-245" target="-KWZgO-KfmSXsMUV4DuB-189" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="510" y="320" />
              <mxPoint x="666" y="320" />
              <mxPoint x="666" y="292" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-249" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="290" y="237" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-254" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="260" y="317" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-284" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-282" target="-KWZgO-KfmSXsMUV4DuB-80" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-282" value="Check&amp;nbsp;neighbor_update" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-285" vertex="1">
          <mxGeometry x="162" y="145" width="148" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-283" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-285" source="-KWZgO-KfmSXsMUV4DuB-76" target="-KWZgO-KfmSXsMUV4DuB-282" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-294" value="" style="group" parent="1" vertex="1" connectable="0">
          <mxGeometry x="1578" y="240" width="750" height="270" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-215" value="" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;rotation=90;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="60" y="154" width="60" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-132" value="" style="rhombus;whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="50" y="149" width="60" height="60" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-133" value="" style="whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry width="750" height="270" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-134" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;find_close_neigboor_2sink&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry width="180" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-136" value="Extract the 6&amp;nbsp;neighbors&lt;br&gt;NO need for S0 ( current position)" style="rounded=0;whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="10" y="50" width="160" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-138" value="Sort the list by distance&amp;nbsp;&lt;br&gt;&lt;b&gt;descending&lt;/b&gt;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="200" y="50" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-135" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-136" target="-KWZgO-KfmSXsMUV4DuB-138" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-140" value="slice the first 3&amp;nbsp;&lt;br&gt;( 3 spots close to Border)&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="330" y="50" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-137" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-138" target="-KWZgO-KfmSXsMUV4DuB-140" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-142" value="Filter the list of the spot to keep only the occupied&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="460" y="50" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-139" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-140" target="-KWZgO-KfmSXsMUV4DuB-142" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-154" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="80" y="209" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-155" value="Retunr ID&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="470" y="140.5" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-152" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-153" target="-KWZgO-KfmSXsMUV4DuB-155" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="220" y="166" />
              <mxPoint x="220" y="212" />
              <mxPoint x="520" y="212" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-156" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="130" y="133" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-157" value="Only one&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="440" y="187" width="70" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-201" value="Exist&lt;br style=&quot;border-color: var(--border-color);&quot;&gt;Irremovable" style="rhombus;whiteSpace=wrap;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="45" y="120" width="90" height="91" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-206" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="255" y="140.5" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-151" value="Find many&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="185" y="133" width="70" height="65" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-149" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-151" target="-KWZgO-KfmSXsMUV4DuB-153" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-207" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-151" target="-KWZgO-KfmSXsMUV4DuB-155" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-205" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-201" target="-KWZgO-KfmSXsMUV4DuB-151" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-208" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="220" y="184" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-209" value="Retunr -1&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="470" y="217" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-216" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-215" target="-KWZgO-KfmSXsMUV4DuB-209" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="90" y="242" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-153" value="Find the closest to Border&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="320" y="140.5" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-292" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;exitX=1;exitY=0.5;exitDx=0;exitDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-286" edge="1">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="655.0000000000005" y="102.99999999999977" as="sourcePoint" />
            <mxPoint x="45" y="168.49999999999955" as="targetPoint" />
            <Array as="points">
              <mxPoint x="740" y="75" />
              <mxPoint x="740" y="113" />
              <mxPoint x="25" y="113" />
              <mxPoint x="25" y="169" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-286" value="Check&amp;nbsp;neighbor_update" style="html=1;dashed=0;whiteSpace=wrap;" parent="-KWZgO-KfmSXsMUV4DuB-294" vertex="1">
          <mxGeometry x="590" y="50" width="130" height="50" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-287" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="-KWZgO-KfmSXsMUV4DuB-294" source="-KWZgO-KfmSXsMUV4DuB-142" target="-KWZgO-KfmSXsMUV4DuB-286" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-258" value="Check&amp;nbsp;neighbor_update" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="950" y="19" width="150" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-265" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;exitX=0.5;exitY=1;exitDx=0;exitDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-266" edge="1">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="1337.4074074074074" y="99.1253380782918" as="targetPoint" />
            <Array as="points">
              <mxPoint x="1139.4074074074074" y="180.47000000000003" />
              <mxPoint x="1338.2222222222222" y="180.47000000000003" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-266" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;&lt;br&gt;&lt;br&gt;listener_neighbor_&lt;br&gt;update_state&lt;br&gt;raised&lt;/font&gt;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1082.37" y="33" width="127.63" height="131.2" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-267" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.033;entryY=0.505;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-262" target="-KWZgO-KfmSXsMUV4DuB-266" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-272" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-268" edge="1">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="1390" y="99" as="targetPoint" />
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-268" value="wait until mutex is unlocked" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1239.9992592592594" y="72.36032028469748" width="81.48148148148148" height="52.480427046263344" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-269" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-266" target="-KWZgO-KfmSXsMUV4DuB-268" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-270" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1178.5185185185185" y="61.86423487544482" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-271" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1140.0040740740742" y="157.3629181494662" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-297" value="" style="endArrow=classic;html=1;rounded=0;entryX=0.223;entryY=0.016;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" target="-KWZgO-KfmSXsMUV4DuB-262" edge="1">
          <mxGeometry width="50" height="50" relative="1" as="geometry">
            <mxPoint x="910" y="50" as="sourcePoint" />
            <mxPoint x="930" y="61.86" as="targetPoint" />
            <Array as="points">
              <mxPoint x="950" y="50" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-301" value="" style="group" parent="1" vertex="1" connectable="0">
          <mxGeometry x="970.7388888888889" y="72.36032028469748" width="81.48333333333323" height="52.480427046263344" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-262" value="" style="shape=hexagon;perimeter=hexagonPerimeter2;whiteSpace=wrap;html=1;size=0.25" parent="-KWZgO-KfmSXsMUV4DuB-301" vertex="1">
          <mxGeometry x="0.0018518518518249039" width="81.48148148148148" height="52.480427046263344" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-264" value="check flage" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="-KWZgO-KfmSXsMUV4DuB-301" vertex="1">
          <mxGeometry y="11.236085409252667" width="80" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-311" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1900" y="740" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-312" value="NO&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1840" y="840" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-316" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=1;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-314" target="-KWZgO-KfmSXsMUV4DuB-227" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-314" value="&lt;font style=&quot;font-size: 14px;&quot;&gt;wait&lt;/font&gt;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1850" y="885.5" width="90" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-372" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-317" target="-KWZgO-KfmSXsMUV4DuB-4" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-373" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-317" target="-KWZgO-KfmSXsMUV4DuB-366" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-317" value="&lt;font style=&quot;font-size: 20px;&quot;&gt;START&lt;/font&gt;" style="html=1;dashed=0;whiteSpace=wrap;shape=mxgraph.dfd.start" parent="1" vertex="1">
          <mxGeometry x="10" y="625" width="110.5" height="47.5" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-368" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-366" target="-KWZgO-KfmSXsMUV4DuB-5" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-366" value="Check&amp;nbsp;neighbor_update" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="173" y="627" width="148" height="45" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-19" value="listener_neighbor_update_state" style="shape=manualInput;whiteSpace=wrap;html=1;dashed=0;size=15;" parent="1" vertex="1">
          <mxGeometry x="-270" y="72.36" width="205" height="60" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-20" value="listener_current_updated_irremovable" style="shape=manualInput;whiteSpace=wrap;html=1;dashed=0;size=15;" parent="1" vertex="1">
          <mxGeometry x="-270" y="142.36" width="205" height="60" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-28" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=1;entryDx=0;entryDy=0;" edge="1" parent="1" source="-KWZgO-KfmSXsMUV4DuB-21" target="k8xDso0bWmZA_ZwyiJWP-15">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-1075" y="270" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-21" value="listener_end_of_spanning" style="shape=manualInput;whiteSpace=wrap;html=1;dashed=0;size=15;" parent="1" vertex="1">
          <mxGeometry x="-270" y="212.36" width="205" height="60" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-47" value="&lt;b&gt;Self.state&amp;nbsp;&amp;nbsp;&lt;br&gt;&lt;/b&gt;Sheared var&lt;br&gt;&lt;br&gt;" style="shape=parallelogram;perimeter=parallelogramPerimeter;whiteSpace=wrap;html=1;dashed=0;" parent="1" vertex="1">
          <mxGeometry x="-121" y="334" width="221" height="53" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-384" value="&lt;font style=&quot;font-size: 25px;&quot;&gt;&lt;b&gt;Main thread&lt;/b&gt;&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1123.52" y="-210" width="160" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-394" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-385" target="-KWZgO-KfmSXsMUV4DuB-393" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-385" value="Decode the REC MSG" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-730" y="-150" width="110" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-387" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-386" target="-KWZgO-KfmSXsMUV4DuB-385" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-386" value="REC MSG&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-840" y="-150" width="80" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-399" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-393" target="-KWZgO-KfmSXsMUV4DuB-398" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-28" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-393" target="PSZtOdJVlawKTp13Huel-27" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-393" value="Data&lt;br&gt;ID is positive&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="-830" y="-58" width="100" height="90" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-396" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-820" y="19" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-397" value="MSG end of phase" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-750" y="212.36" width="120" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-402" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-398" target="-KWZgO-KfmSXsMUV4DuB-401" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-11" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-398" target="PSZtOdJVlawKTp13Huel-3" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-398" value="Is it equal to ID&lt;br&gt;&amp;nbsp;of current drone&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="-700" y="-66" width="120" height="106" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-400" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-740" y="-40" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-404" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-401" target="-KWZgO-KfmSXsMUV4DuB-403" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-401" value="Lock state var" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-675" y="113.6" width="70" height="34" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-405" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.067;entryY=0.662;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-403" target="-KWZgO-KfmSXsMUV4DuB-47" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-140" y="370" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-408" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.25;entryY=1;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-403" target="-KWZgO-KfmSXsMUV4DuB-47" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-500" y="420" />
              <mxPoint x="-66" y="420" />
            </Array>
            <mxPoint x="-130" y="390" as="targetPoint" />
          </mxGeometry>
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-9" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-403" target="-KWZgO-KfmSXsMUV4DuB-413" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-403" value="is it Border" style="rhombus;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-540" y="330" width="80" height="80" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-406" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-460" y="345.5" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-407" value="set to Irremovable_boarder&lt;span style=&quot;background-color: initial;&quot;&gt;&amp;nbsp;&lt;/span&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-370" y="334" width="170" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-409" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-460" y="400" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-410" value="&lt;span style=&quot;color: rgb(0, 0, 0); font-family: Helvetica; font-size: 12px; font-style: normal; font-variant-ligatures: normal; font-variant-caps: normal; font-weight: 400; letter-spacing: normal; orphans: 2; text-align: center; text-indent: 0px; text-transform: none; widows: 2; word-spacing: 0px; -webkit-text-stroke-width: 0px; background-color: rgb(251, 251, 251); text-decoration-thickness: initial; text-decoration-style: initial; text-decoration-color: initial; float: none; display: inline !important;&quot;&gt;set to Irremovable&lt;/span&gt;" style="text;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-360" y="395" width="100" height="40" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-2" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-413" target="-KWZgO-KfmSXsMUV4DuB-20" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-413" value="Raise&amp;nbsp; flag&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="-470" y="154.86" width="60" height="35" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-416" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-640" y="49" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-1" value="&lt;font style=&quot;font-size: 10px;&quot;&gt;Declear changes in&lt;br&gt;current&amp;nbsp; neighbor_list&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-600" y="-58" width="120" height="40" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-4" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="PSZtOdJVlawKTp13Huel-3" target="-KWZgO-KfmSXsMUV4DuB-19" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-450" y="102" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-16" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="PSZtOdJVlawKTp13Huel-3" target="PSZtOdJVlawKTp13Huel-15" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-3" value="Raise &lt;br&gt;flag&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-480" y="-28" width="60" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-5" value="To prevent Mian to&amp;nbsp; acces&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-440" y="72.36" width="160" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-10" value="&lt;b&gt;Neighbor_list&lt;/b&gt;&lt;br&gt;Sheared var&lt;br&gt;" style="shape=parallelogram;perimeter=parallelogramPerimeter;whiteSpace=wrap;html=1;dashed=0;" parent="1" vertex="1">
          <mxGeometry x="-130" y="-150" width="221" height="53" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-12" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-590" y="-11" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-18" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.25;entryDx=0;entryDy=0;" parent="1" source="PSZtOdJVlawKTp13Huel-15" target="PSZtOdJVlawKTp13Huel-10" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-340" y="-133" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-24" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="PSZtOdJVlawKTp13Huel-15" target="PSZtOdJVlawKTp13Huel-22" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-15" value="chang the states&lt;br&gt;of drone with ID REC" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-390" y="-43" width="100" height="60" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-19" value="&lt;span style=&quot;color: rgb(0, 0, 0); font-family: Helvetica; font-size: 12px; font-style: normal; font-variant-ligatures: normal; font-variant-caps: normal; font-weight: 400; letter-spacing: normal; orphans: 2; text-align: center; text-indent: 0px; text-transform: none; widows: 2; word-spacing: 0px; -webkit-text-stroke-width: 0px; background-color: rgb(251, 251, 251); text-decoration-thickness: initial; text-decoration-style: initial; text-decoration-color: initial; float: none; display: inline !important;&quot;&gt;set to Irremovable&lt;/span&gt;" style="text;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-270" y="-160" width="100" height="40" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-23" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" parent="1" source="PSZtOdJVlawKTp13Huel-22" target="-KWZgO-KfmSXsMUV4DuB-19" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-22" value="Clear&lt;br&gt;flag&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-260" y="-28" width="60" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-26" value="&lt;span style=&quot;color: rgb(0, 0, 0); font-family: Helvetica; font-size: 12px; font-style: normal; font-variant-ligatures: normal; font-variant-caps: normal; font-weight: 400; letter-spacing: normal; orphans: 2; text-align: center; text-indent: 0px; text-transform: none; widows: 2; word-spacing: 0px; -webkit-text-stroke-width: 0px; background-color: rgb(251, 251, 251); text-decoration-thickness: initial; text-decoration-style: initial; text-decoration-color: initial; float: none; display: inline !important;&quot;&gt;Allow Mian to&amp;nbsp; acces&amp;nbsp;&lt;/span&gt;" style="text;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-217.5" y="14" width="170" height="40" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-29" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="PSZtOdJVlawKTp13Huel-27" target="-KWZgO-KfmSXsMUV4DuB-21" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-27" value="Raise&amp;nbsp; flag&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="-810" y="224.86" width="60" height="35" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-30" value="&lt;b&gt;&lt;font style=&quot;font-size: 25px;&quot;&gt;&amp;nbsp;Listening thread&amp;nbsp;&lt;/font&gt;&lt;/b&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-720" y="-215" width="230" height="40" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-3" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="PSZtOdJVlawKTp13Huel-36" target="vDaD4nOrAfAWEzzxnboz-2" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-36" value="REC MSG" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-830" y="894" width="120" height="65" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-3" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="vDaD4nOrAfAWEzzxnboz-2" target="2TuK2keB_ZSRptDu4ZPO-2" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-2" value="Decode the REC MSG" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-670" y="895.5" width="130" height="60" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-10" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-2" target="2TuK2keB_ZSRptDu4ZPO-6" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-2" value="ID != Sink ID" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="-500" y="868.5" width="110" height="114" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-4" value="Means a neighbor became Irremovable &lt;br&gt;so a path to sink is formed&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-530" y="981.5" width="230" height="40" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-20" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-6" target="2TuK2keB_ZSRptDu4ZPO-12" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-180" y="927" />
              <mxPoint x="-180" y="960" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-34" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-6" target="2TuK2keB_ZSRptDu4ZPO-11" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="-180" y="927" />
              <mxPoint x="-180" y="880" />
              <mxPoint x="-88" y="880" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-6" value="set lock&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="-330" y="901.5" width="85" height="50" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-11" value="&lt;b&gt;message_counter&lt;/b&gt;&lt;br&gt;Sheared var&lt;br&gt;" style="shape=parallelogram;perimeter=parallelogramPerimeter;whiteSpace=wrap;html=1;dashed=0;" parent="1" vertex="1">
          <mxGeometry x="-105" y="854" width="221" height="53" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-12" value="&lt;b&gt;remaining_time&lt;/b&gt;&lt;br&gt;Sheared var&lt;br&gt;" style="shape=parallelogram;perimeter=parallelogramPerimeter;whiteSpace=wrap;html=1;dashed=0;" parent="1" vertex="1">
          <mxGeometry x="-130" y="925.5" width="221" height="53" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-22" value="Increment&amp;nbsp;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-172.5" y="840" width="80" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-23" value="Rest to orginal timeout" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-215" y="978.5" width="140" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-25" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-390" y="928" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-26" value="&lt;b&gt;&lt;font style=&quot;font-size: 25px;&quot;&gt;Sink listener&lt;/font&gt;&lt;/b&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="-680" y="1250" width="170" height="40" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-31" value="" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="80" y="995" width="1630" height="265" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-35" value="Create a object of class Timer&amp;nbsp;&lt;br&gt;&lt;br&gt;Def time of wait(timeout)&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="99.999209486166" y="1044.8063687150839" width="149.32806324110672" height="70.39106145251397" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-38" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="vDaD4nOrAfAWEzzxnboz-4" target="2TuK2keB_ZSRptDu4ZPO-36" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-4" value="" style="html=1;dashed=0;whiteSpace=wrap;shape=mxgraph.dfd.loop" parent="1" vertex="1">
          <mxGeometry x="290" y="1011.12" width="700" height="108.88" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-22" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="PSZtOdJVlawKTp13Huel-35" target="vDaD4nOrAfAWEzzxnboz-4" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-67" value="" style="group" parent="1" vertex="1" connectable="0">
          <mxGeometry x="380.8953096179183" y="1019.9982681564246" width="589.8458498023715" height="98.04469273743017" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-5" value="sleep eps" style="html=1;dashed=0;whiteSpace=wrap;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="12.444005270092227" y="27.150837988826815" width="49.776021080368906" height="30.16759776536313" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-20" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="vDaD4nOrAfAWEzzxnboz-67" source="vDaD4nOrAfAWEzzxnboz-8" target="vDaD4nOrAfAWEzzxnboz-5" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="304.87812911725956" y="92.51396648044692" />
              <mxPoint y="92.51396648044692" />
              <mxPoint y="42.234636871508386" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-8" value="&lt;br&gt;remaining_time&amp;nbsp;&lt;br&gt;&amp;lt;=0" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="248.88010540184453" width="111.99604743083003" height="80.44692737430168" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-10" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.049;entryY=0.529;entryDx=0;entryDy=0;entryPerimeter=0;" parent="vDaD4nOrAfAWEzzxnboz-67" source="vDaD4nOrAfAWEzzxnboz-9" target="vDaD4nOrAfAWEzzxnboz-8" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-9" value="remaining_time - =0.5" style="html=1;dashed=0;whiteSpace=wrap;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="99.55204216073781" y="17.59776536312849" width="124.44005270092227" height="50.27932960893855" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-11" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="vDaD4nOrAfAWEzzxnboz-67" source="vDaD4nOrAfAWEzzxnboz-5" target="vDaD4nOrAfAWEzzxnboz-9" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-12" value="time_up()&lt;br&gt;bordcast end of spanning&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="398.20816864295125" y="15.083798882681565" width="113.24044795783927" height="50.27932960893855" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-13" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="vDaD4nOrAfAWEzzxnboz-67" source="vDaD4nOrAfAWEzzxnboz-8" target="vDaD4nOrAfAWEzzxnboz-12" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-14" value="break" style="html=1;dashed=0;whiteSpace=wrap;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="540.0698287220026" y="25.139664804469277" width="49.776021080368906" height="30.16759776536313" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-16" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="vDaD4nOrAfAWEzzxnboz-67" source="vDaD4nOrAfAWEzzxnboz-12" target="vDaD4nOrAfAWEzzxnboz-14" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-17" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="335.9881422924901" y="7.039106145251397" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="vDaD4nOrAfAWEzzxnboz-21" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="vDaD4nOrAfAWEzzxnboz-67" vertex="1">
          <mxGeometry x="298.65612648221344" y="67.87709497206704" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="PSZtOdJVlawKTp13Huel-34" value="&lt;font style=&quot;font-size: 14px;&quot;&gt;spanning_sink&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="91" y="1000" width="110" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-27" value="&lt;font style=&quot;font-size: 12px;&quot;&gt;Lunch Sink listener thread&amp;nbsp;&lt;/font&gt;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="99.33" y="1115.2" width="150" height="42" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-33" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.25;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-28" target="vDaD4nOrAfAWEzzxnboz-5" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-28" value="set lock&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="308" y="1030" width="54.1" height="35" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-50" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-36" target="2TuK2keB_ZSRptDu4ZPO-45" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-56" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-36" target="2TuK2keB_ZSRptDu4ZPO-42" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-36" value="&lt;br&gt;message_counter ==0&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1026" y="1011.12" width="114" height="110" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-40" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.25;entryY=0;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-12" target="vDaD4nOrAfAWEzzxnboz-9" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-41" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-11" target="2TuK2keB_ZSRptDu4ZPO-36" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="90" y="920" />
              <mxPoint x="1083" y="920" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-27" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=1;entryY=0.5;entryDx=0;entryDy=0;" edge="1" parent="1" source="2TuK2keB_ZSRptDu4ZPO-42" target="k8xDso0bWmZA_ZwyiJWP-25">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1083" y="1220" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-42" value="Send MSG = -127&lt;br&gt;End of spanning" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="1024.5" y="1150" width="117" height="40" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-58" value="" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1040" y="661.5" width="660" height="224" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-65" value="&lt;font style=&quot;font-size: 13px;&quot;&gt;build_path ()&amp;nbsp;&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1033" y="656.5" width="100" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-159" value="Call&lt;br&gt;&amp;nbsp;find_close_neigboor_2sink" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1045" y="694.5" width="150" height="51" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-3" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" edge="1" parent="1" source="-KWZgO-KfmSXsMUV4DuB-160" target="k8xDso0bWmZA_ZwyiJWP-2">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-160" value="return&amp;nbsp;&lt;br&gt;positive (id)&lt;br&gt;&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1220" y="684" width="85" height="70" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-161" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.043;entryY=0.502;entryDx=0;entryDy=0;entryPerimeter=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-159" target="-KWZgO-KfmSXsMUV4DuB-160" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-162" value="send MSG contains ID&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1407" y="696" width="100" height="46" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-166" value="Drone is&amp;nbsp;&lt;br&gt;Irremovable&lt;br&gt;and border" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1540" y="672.5" width="100" height="93" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-178" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-162" target="-KWZgO-KfmSXsMUV4DuB-166" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-168" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1640" y="706" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-169" value="Call&lt;br&gt;&amp;nbsp;find_close_neigboor_2border" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1150" y="798" width="160" height="51" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-180" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;exitX=0.5;exitY=1;exitDx=0;exitDy=0;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-166" target="-KWZgO-KfmSXsMUV4DuB-169" edge="1">
          <mxGeometry relative="1" as="geometry">
            <Array as="points">
              <mxPoint x="1560" y="778.5" />
              <mxPoint x="1130" y="778.5" />
              <mxPoint x="1130" y="823.5" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-6" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" edge="1" parent="1" source="-KWZgO-KfmSXsMUV4DuB-170" target="k8xDso0bWmZA_ZwyiJWP-5">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-170" value="return&amp;nbsp;&lt;br&gt;positive (id)&lt;br&gt;&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1345" y="788.5" width="85" height="70" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-172" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="-KWZgO-KfmSXsMUV4DuB-169" target="-KWZgO-KfmSXsMUV4DuB-170" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-171" value="send MSG contains ID&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1550" y="800.5" width="100" height="46" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-179" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1505" y="754.5" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="-KWZgO-KfmSXsMUV4DuB-181" value="&lt;font style=&quot;font-size: 10px;&quot;&gt;No need for path &lt;br&gt;to border&amp;nbsp;&lt;/font&gt;" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1600" y="670" width="100" height="40" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-45" value="Call&lt;br&gt;&amp;nbsp;find_close_neigboor_2border" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1180" y="1040.31" width="160" height="51" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-46" value="return&amp;nbsp;&lt;br&gt;positive (id)&lt;br&gt;&amp;nbsp;" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" parent="1" vertex="1">
          <mxGeometry x="1370" y="1034.02" width="85" height="70" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-47" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-45" target="2TuK2keB_ZSRptDu4ZPO-46" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-59" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=1;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-48" target="2TuK2keB_ZSRptDu4ZPO-57" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-48" value="send MSG contains ID&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" parent="1" vertex="1">
          <mxGeometry x="1480" y="1044.81" width="100" height="46" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-49" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-46" target="2TuK2keB_ZSRptDu4ZPO-48" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-51" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1125" y="1035" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-52" value="NO" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1090" y="1110" width="40" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-53" value="Path found" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" parent="1" vertex="1">
          <mxGeometry x="1002.37" y="1110" width="80" height="30" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-60" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" parent="1" source="2TuK2keB_ZSRptDu4ZPO-57" target="2TuK2keB_ZSRptDu4ZPO-42" edge="1">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="2TuK2keB_ZSRptDu4ZPO-57" value="wait until a message back from border through the path&amp;nbsp;" style="rounded=0;whiteSpace=wrap;html=1;" parent="1" vertex="1">
          <mxGeometry x="1465" y="1140" width="120" height="60" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-4" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-2" target="-KWZgO-KfmSXsMUV4DuB-162">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-2" value="save the id in var" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="1330" y="694.5" width="58.52" height="50.25" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-7" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-5" target="-KWZgO-KfmSXsMUV4DuB-171">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-5" value="save the id in var" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="1455" y="798.5" width="58.52" height="50.25" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-8" value="Loop" style="text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;whiteSpace=wrap;rounded=0;" vertex="1" parent="1">
          <mxGeometry x="704" y="420" width="60" height="30" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-13" value="&lt;font style=&quot;font-size: 17px;&quot;&gt;Loop&lt;/font&gt;" style="text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;whiteSpace=wrap;rounded=0;" vertex="1" parent="1">
          <mxGeometry x="-1050" y="-190" width="60" height="30" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-19" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0;entryY=0.5;entryDx=0;entryDy=0;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-15" target="k8xDso0bWmZA_ZwyiJWP-17">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-15" value="" style="shape=hexagon;perimeter=hexagonPerimeter2;whiteSpace=wrap;html=1;size=0.25" vertex="1" parent="1">
          <mxGeometry x="-1140" y="110" width="130" height="50" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-16" value="check&amp;nbsp;&lt;br&gt;end_of_spanning" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" vertex="1" parent="1">
          <mxGeometry x="-1135" y="110.60000000000001" width="120" height="40" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-23" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-17" target="k8xDso0bWmZA_ZwyiJWP-20">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-17" value="is raised" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" vertex="1" parent="1">
          <mxGeometry x="-980" y="95.00000000000001" width="80" height="80" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-20" value="Break the loop" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="-990" y="202.36" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-24" value="YES" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" vertex="1" parent="1">
          <mxGeometry x="-1000" y="157.36" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-36" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=1;entryDx=0;entryDy=0;" edge="1" parent="1" target="k8xDso0bWmZA_ZwyiJWP-30">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="115.99999999999955" y="1235.01" as="sourcePoint" />
            <mxPoint x="-1070" y="954.99" as="targetPoint" />
            <Array as="points">
              <mxPoint x="-1065" y="1235" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-25" value="sink_t.end_of_spanning" style="shape=manualInput;whiteSpace=wrap;html=1;dashed=0;size=15;" vertex="1" parent="1">
          <mxGeometry x="116" y="1190" width="205" height="60" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-29" value="SET" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" vertex="1" parent="1">
          <mxGeometry x="340" y="1190" width="50" height="30" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-30" value="" style="shape=hexagon;perimeter=hexagonPerimeter2;whiteSpace=wrap;html=1;size=0.25" vertex="1" parent="1">
          <mxGeometry x="-1130" y="883.5" width="130" height="50" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-34" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-31" target="k8xDso0bWmZA_ZwyiJWP-32">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-31" value="check&amp;nbsp;&lt;br&gt;end_of_spanning" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" vertex="1" parent="1">
          <mxGeometry x="-1130" y="888.5" width="120" height="40" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-38" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-32" target="k8xDso0bWmZA_ZwyiJWP-33">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-32" value="is raised" style="shape=rhombus;html=1;dashed=0;whiteSpace=wrap;perimeter=rhombusPerimeter;" vertex="1" parent="1">
          <mxGeometry x="-970" y="868.5" width="80" height="80" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-40" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;entryX=0.5;entryY=0;entryDx=0;entryDy=0;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-33" target="k8xDso0bWmZA_ZwyiJWP-39">
          <mxGeometry relative="1" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-33" value="Break the loop" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="-980" y="1000" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-41" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;" edge="1" parent="1" source="k8xDso0bWmZA_ZwyiJWP-39">
          <mxGeometry relative="1" as="geometry">
            <mxPoint x="120" y="1220" as="targetPoint" />
            <Array as="points">
              <mxPoint x="-382" y="1136" />
              <mxPoint x="-382" y="1220" />
            </Array>
          </mxGeometry>
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-39" value="Clear the flag&amp;nbsp;&lt;br&gt;( for next iteration)&amp;nbsp;" style="html=1;dashed=0;whiteSpace=wrap;" vertex="1" parent="1">
          <mxGeometry x="-980" y="1111.2" width="100" height="50" as="geometry" />
        </mxCell>
        <mxCell id="k8xDso0bWmZA_ZwyiJWP-42" value="CLEAR" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;" vertex="1" parent="1">
          <mxGeometry x="-370" y="1180" width="60" height="30" as="geometry" />
        </mxCell>
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
@enduml

## How to find tagret in the region of the drone
The drone will scan the region by taking a hexagon path with length depends on the camera setting.
For example based on the focal length and the high of the camera, the covrage will be calculate and define the path of the drone.

```bash
        scan_hexagon(vehicle-object, VESPA-object, orgin-hex-length ,camera_image_width, scan_time)
```

- It will start by calculating the distance between two successive hexagons that confine between them the size of the image that can be captured by the drone
- calcule Distance/2 beause the drone should be in the middle.
- scan the first region
    - go north (y) to V'1= (V1-distance/2)
    - move on the path that form hexagon
    - finish the loop
- scan the second region
    - from the last position (v'1) go down by distance to v''1=( V'1-distance)
    - move on the path and scan a hexagon
    - Note: the reason of moving by "distance" not "distance/2" because (distance/2) was used to go to the path of the drone, then the next path is on "distane" as you see in the fig.
- repeat until can't subtract (distance) which means no more regions need to be scanned.
- suppose the last hexagon was the one with (v'''1), then the drone can back to the original center
by going south (a-distance*i+distance/2)
![Alt text](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/.exp/coverage.png)
