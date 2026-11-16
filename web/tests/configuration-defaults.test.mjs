import test from 'node:test';
import assert from 'node:assert/strict';
import { createDefaultConfig } from '../app/configuration-forms.js';

const topic = (name, type_name = 'sensor_msgs/CompressedImage', extra = {}) => ({ name, type_name, status: 'ok', header_time_valid: true, message_count: 12, ...extra });

test('unordered topics select every usable camera with numeric path ordering', () => {
  const topics = [topic('/camera10/image'), topic('/camera2/image'), topic('/camera0/image'), topic('/bad/image', undefined, { header_time_valid: false }), topic('/empty/image', undefined, { message_count: 0 }), topic('/notes', 'std_msgs/String')];
  const config = createDefaultConfig({ topics });
  assert.deepEqual(config.cameras.map(({ id, topic }) => [id, topic]), [['cam0', '/camera0/image'], ['cam1', '/camera2/image'], ['cam2', '/camera10/image']]);
  assert.equal(config.calibration.type, 'multi_camera');
  assert.equal(config.calibration.reference_camera, 'cam0');
  assert.equal(topics[0].name, '/camera10/image');
});

test('first usable IMU by natural path order selects joint calibration', () => {
  const config = createDefaultConfig({ topics: [topic('/cam1/image'), topic('/imu10', 'sensor_msgs/Imu'), topic('/imu2', 'sensor_msgs/Imu'), topic('/imu0', 'sensor_msgs/Imu', { status: 'decode failed' }), topic('/cam0/image')] });
  assert.equal(config.source.imu_topic, '/imu2');
  assert.equal(config.calibration.type, 'multi_camera_imu');
  assert.equal(config.calibration.visual_reference_camera, config.cameras[0].id);
});

test('single camera without usable IMU infers the single-camera contract', () => {
  const config = createDefaultConfig({ topics: [topic('/camera/image', 'sensor_msgs/Image'), topic('/imu', 'sensor_msgs/Imu', { message_count: 0 })] });
  assert.equal(config.source.imu_topic, null);
  assert.equal(config.calibration.type, 'single_camera');
  assert.equal(config.calibration.camera, config.cameras[0].id);
});
