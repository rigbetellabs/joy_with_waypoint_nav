# joy_with_waypoint_nav
Joy teleop package with waypoint based autonomous navigation using joy buttons

```
     <param name="axis_linear" value="1" type="int"/>
     <param name="axis_angular" value="0" type="int"/>
     <param name="scale_linear" value="-0.2" type="double"/>
     <param name="scale_angular" value="0.4" type="double"/>
     <node pkg="auto_joy_teleop" type="auto_joy_teleop" name="auto_joy_teleop"/>
```

![autojoy](autojoyteleop.png)

### Primary Issues with `/joy/feedback`

- Controller responds to the published `/joy/feedback` messages only after 1 seconds
- Duration between the two adjacent messages should be atleast 1 second, if duration between any two adjacent messages is less than 1 second the latter one will not be reflected on controller.