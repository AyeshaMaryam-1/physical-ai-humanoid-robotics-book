using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2UnityEx;
using std_msgs;
using sensor_msgs;

public class RosJointStateSubscriber : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeServerUrl = "ws://127.0.0.1:9090";

    [Header("Joint Mapping")]
    public JointMapping[] jointMappings;

    private Ros2UnityComponent ros2Unity;
    private bool connected = false;

    // Start is called before the first frame update
    void Start()
    {
        InitializeRosConnection();
    }

    void InitializeRosConnection()
    {
        ros2Unity = GetComponent<Ros2UnityComponent>();
        ros2Unity.InitialiseRosConnection(rosBridgeServerUrl);

        // Create subscription to joint states
        ros2Unity.CreateSubscription<JointStateMsg>("joint_states",
            (JointStateMsg msg) => OnJointStatesReceived(msg));

        connected = true;
        Debug.Log("Connected to ROS Bridge: " + rosBridgeServerUrl);
    }

    void OnJointStatesReceived(JointStateMsg jointStateMsg)
    {
        if (jointStateMsg.name.Count != jointStateMsg.position.Count)
        {
            Debug.LogWarning("Joint names and positions count mismatch!");
            return;
        }

        // Update each joint based on received positions
        for (int i = 0; i < jointStateMsg.name.Count; i++)
        {
            string jointName = jointStateMsg.name[i];
            float jointPosition = jointStateMsg.position[i];

            // Find corresponding joint in our mapping
            foreach (JointMapping mapping in jointMappings)
            {
                if (mapping.rosJointName == jointName)
                {
                    UpdateJoint(mapping, jointPosition);
                    break;
                }
            }
        }
    }

    void UpdateJoint(JointMapping mapping, float position)
    {
        if (mapping.jointTransform == null) return;

        // Apply joint position based on joint type
        switch (mapping.jointType)
        {
            case JointType.Revolute:
            case JointType.Continuous:
                // For revolute joints, apply rotation
                Vector3 rotation = Vector3.zero;
                switch (mapping.rotationAxis)
                {
                    case RotationAxis.X:
                        rotation = new Vector3(position * Mathf.Rad2Deg, 0, 0);
                        break;
                    case RotationAxis.Y:
                        rotation = new Vector3(0, position * Mathf.Rad2Deg, 0);
                        break;
                    case RotationAxis.Z:
                        rotation = new Vector3(0, 0, position * Mathf.Rad2Deg);
                        break;
                }
                mapping.jointTransform.localRotation = Quaternion.Euler(rotation);
                break;

            case JointType.Prismatic:
                // For prismatic joints, apply translation
                Vector3 translation = Vector3.zero;
                switch (mapping.translationAxis)
                {
                    case TranslationAxis.X:
                        translation = new Vector3(position, 0, 0);
                        break;
                    case TranslationAxis.Y:
                        translation = new Vector3(0, position, 0);
                        break;
                    case TranslationAxis.Z:
                        translation = new Vector3(0, 0, position);
                        break;
                }
                mapping.jointTransform.localPosition = translation;
                break;
        }
    }

    void Update()
    {
        if (ros2Unity != null && connected)
        {
            ros2Unity.Update();
        }
    }

    void OnDestroy()
    {
        if (ros2Unity != null)
        {
            ros2Unity.Dispose();
        }
    }
}

// Helper enums for joint types
public enum JointType
{
    Revolute,
    Continuous,
    Prismatic,
    Fixed
}

public enum RotationAxis
{
    X,
    Y,
    Z
}

public enum TranslationAxis
{
    X,
    Y,
    Z
}

// Helper class to map ROS joints to Unity transforms
[System.Serializable]
public class JointMapping
{
    [Header("ROS Joint Info")]
    public string rosJointName;
    public JointType jointType = JointType.Revolute;

    [Header("Unity Transform")]
    public Transform jointTransform;

    [Header("Joint Configuration")]
    public RotationAxis rotationAxis = RotationAxis.Y;
    public TranslationAxis translationAxis = TranslationAxis.X;
}