'use strict';

const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('subscription_example_node');

  node.createSubscription('std_msgs/msg/String', 'wot_topic', (msg) => {
    console.log(`Received message: `, msg);
  });

  rclnodejs.spin(node);
});