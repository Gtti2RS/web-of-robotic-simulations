// test.js
const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');

  let counter = 0;
  setInterval(() => {
    console.log(`Publishing message: Hello ROS ${counter}`);
    publisher.publish(`Hello ROS ${counter++}`);
  }, 1000);

  rclnodejs.spin(node);
});
