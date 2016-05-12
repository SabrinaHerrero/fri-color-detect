/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include <PololuLedStrip.h>
#include <ros.h>
#include <std_msgs/ColorRGBA.h>

#define LED_SIGNAL_PIN 12
PololuLedStrip<LED_SIGNAL_PIN> ledStrip;

#define LED_COUNT 60
rgb_color colors[LED_COUNT];

ros::NodeHandle  nh;

void colorCb( const std_msgs::ColorRGBA& msg){
  rgb_color color;
  color.red = msg.r;
  color.green = msg.g;
  color.blue = msg.b;
  
  // Update the colors buffer.
  for(int i = 0; i < LED_COUNT; i++)
  {
      colors[i] = color;
  }
  // Write to the LED strip.
  ledStrip.write(colors, LED_COUNT);  
}

ros::Subscriber<std_msgs::ColorRGBA> sub("color_chatter", &colorCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
