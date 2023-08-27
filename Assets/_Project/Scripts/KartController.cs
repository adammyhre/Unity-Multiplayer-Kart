using System.Collections.Generic;
using System.Linq;
using Cinemachine;
using Unity.Netcode;
using UnityEngine;
using Utilities;

namespace Kart {
    [System.Serializable]
    public class AxleInfo {
        public WheelCollider leftWheel;
        public WheelCollider rightWheel;
        public bool motor;
        public bool steering;
        public WheelFrictionCurve originalForwardFriction;
        public WheelFrictionCurve originalSidewaysFriction;
    }
    
    // Network variables should be value objects
    public struct InputPayload : INetworkSerializable {
        public int tick;
        public Vector3 inputVector;
        
        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter {
            serializer.SerializeValue(ref tick);
            serializer.SerializeValue(ref inputVector);
        }
    }

    public struct StatePayload : INetworkSerializable {
        public int tick;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        
        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter {
            serializer.SerializeValue(ref tick);
            serializer.SerializeValue(ref position);
            serializer.SerializeValue(ref rotation);
            serializer.SerializeValue(ref velocity);
            serializer.SerializeValue(ref angularVelocity);
        }
    }

    public class KartController : NetworkBehaviour {
        [Header("Axle Information")]
        [SerializeField] AxleInfo[] axleInfos;

        [Header("Motor Attributes")] 
        [SerializeField] float maxMotorTorque = 3000f;
        [SerializeField] float maxSpeed;

        [Header("Steering Attributes")]
        [SerializeField] float maxSteeringAngle = 30f;
        [SerializeField] AnimationCurve turnCurve;
        [SerializeField] float turnStrength = 1500f;
        
        [Header("Braking and Drifting")]
        [SerializeField] float driftSteerMultiplier = 1.5f; // Change in steering during a drift
        [SerializeField] float brakeTorque = 10000f;

        [Header("Physics")]
        [SerializeField] Transform centerOfMass;
        [SerializeField] float downForce = 100f;
        [SerializeField] float gravity = Physics.gravity.y;
        [SerializeField] float lateralGScale = 10f; // Scaling factor for lateral G forces;

        [Header("Banking")]
        [SerializeField] float maxBankAngle = 5f;
        [SerializeField] float bankSpeed = 2f;

        [Header("Refs")] 
        [SerializeField] InputReader playerInput;
        [SerializeField] Circuit circuit;
        [SerializeField] AIDriverData driverData;
        [SerializeField] CinemachineVirtualCamera playerCamera;
        [SerializeField] AudioListener playerAudioListener;
        
        IDrive input;
        Rigidbody rb;
        
        Vector3 kartVelocity;
        float brakeVelocity;
        float driftVelocity;
        
        RaycastHit hit;

        const float thresholdSpeed = 10f;
        const float centerOfMassOffset = -0.5f;
        Vector3 originalCenterOfMass;

        public bool IsGrounded = true;
        public Vector3 Velocity => kartVelocity;
        public float MaxSpeed => maxSpeed;
        
        // Netcode general
        NetworkTimer timer;
        const float k_serverTickRate = 60f; // 60 FPS
        const int k_bufferSize = 1024;
        
        // Netcode client specific
        CircularBuffer<StatePayload> clientStateBuffer;
        CircularBuffer<InputPayload> clientInputBuffer;
        StatePayload lastServerState;
        StatePayload lastProcessedState;
        
        // Netcode server specific
        CircularBuffer<StatePayload> serverStateBuffer;
        Queue<InputPayload> serverInputQueue;
        
        [Header("Netcode")]
        [SerializeField] float reconciliationThreshold = 10f;
        [SerializeField] GameObject serverCube;
        [SerializeField] GameObject clientCube;

        void Awake() {
            if (playerInput is IDrive driveInput) {
                input = driveInput;
            }
            
            rb = GetComponent<Rigidbody>(); 
            input.Enable();
            
            rb.centerOfMass = centerOfMass.localPosition;
            originalCenterOfMass = centerOfMass.localPosition;
            
            foreach (AxleInfo axleInfo in axleInfos) {
                axleInfo.originalForwardFriction = axleInfo.leftWheel.forwardFriction;
                axleInfo.originalSidewaysFriction = axleInfo.leftWheel.sidewaysFriction;
            }
            
            timer = new NetworkTimer(k_serverTickRate);
            clientStateBuffer = new CircularBuffer<StatePayload>(k_bufferSize);
            clientInputBuffer = new CircularBuffer<InputPayload>(k_bufferSize);
            
            serverStateBuffer = new CircularBuffer<StatePayload>(k_bufferSize);
            serverInputQueue = new Queue<InputPayload>();
        }
        
        public void SetInput(IDrive input) {
            this.input = input;
        }
        
        public override void OnNetworkSpawn() {
            if (!IsOwner) {
                playerAudioListener.enabled = false;
                playerCamera.Priority = 0;
                return;
            }
            
            playerCamera.Priority = 100;
            playerAudioListener.enabled = true;
        }

        void Update() {
            timer.Update(Time.deltaTime);
            if (Input.GetKeyDown(KeyCode.Q)) {
                transform.position += transform.forward * 20f;
            }
        }

        void FixedUpdate() {
            if (!IsOwner) return;

            while (timer.ShouldTick()) {
                HandleClientTick();
                HandleServerTick();
            }
        }

        void HandleServerTick() {
            var bufferIndex = -1;
            while (serverInputQueue.Count > 0) {
                InputPayload inputPayload = serverInputQueue.Dequeue();
                
                bufferIndex = inputPayload.tick % k_bufferSize;
                
                var previousBufferIndex = bufferIndex - 1;
                if (previousBufferIndex < 0) previousBufferIndex = k_bufferSize - 1;
                
                StatePayload statePayload = SimulateMovement(inputPayload);
                serverStateBuffer.Add(statePayload, bufferIndex);
            }
            
            if (bufferIndex == -1) return;
            SendToClientRpc(serverStateBuffer.Get(bufferIndex));
        }

        StatePayload SimulateMovement(InputPayload inputPayload) {
            Physics.simulationMode = SimulationMode.Script;
            
            Move(inputPayload.inputVector);
            Physics.Simulate(Time.fixedDeltaTime);
            Physics.simulationMode = SimulationMode.FixedUpdate;
            
            return new StatePayload() {
                tick = inputPayload.tick,
                position = transform.position,
                rotation = transform.rotation,
                velocity = rb.velocity,
                angularVelocity = rb.angularVelocity
            };
        }

        [ClientRpc]
        void SendToClientRpc(StatePayload statePayload) {
            if (!IsOwner) return;
            lastServerState = statePayload;
        }

        void HandleClientTick() {
            if (!IsClient) return;

            var currentTick = timer.CurrentTick;
            var bufferIndex = currentTick % k_bufferSize;
            
            InputPayload inputPayload = new InputPayload() {
                tick = currentTick,
                inputVector = input.Move
            };
            
            clientInputBuffer.Add(inputPayload, bufferIndex);
            SendToServerRpc(inputPayload);
            
            StatePayload statePayload = ProcessMovement(inputPayload);
            clientStateBuffer.Add(statePayload, bufferIndex);
            
            HandleServerReconciliation();
        }

        bool ShouldReconcile() {
            bool isNewServerState = !lastServerState.Equals(default);
            bool isLastStateUndefinedOrDifferent = lastProcessedState.Equals(default) 
                                                   || !lastProcessedState.Equals(lastServerState);
            
            return isNewServerState && isLastStateUndefinedOrDifferent;
        }

        void HandleServerReconciliation() {
            if (!ShouldReconcile()) return;

            float positionError;
            int bufferIndex;
            StatePayload rewindState = default;
            
            bufferIndex = lastServerState.tick % k_bufferSize;
            if (bufferIndex - 1 < 0) return; // Not enough information to reconcile
            
            rewindState = IsHost ? serverStateBuffer.Get(bufferIndex - 1) : lastServerState; // Host RPCs execute immediately, so we can use the last server state
            positionError = Vector3.Distance(rewindState.position, clientStateBuffer.Get(bufferIndex).position);

            if (positionError > reconciliationThreshold) {
                ReconcileState(rewindState);
            }

            lastProcessedState = lastServerState;
        }

        void ReconcileState(StatePayload rewindState) {
            transform.position = rewindState.position;
            transform.rotation = rewindState.rotation;
            rb.velocity = rewindState.velocity;
            rb.angularVelocity = rewindState.angularVelocity;

            if (!rewindState.Equals(lastServerState)) return;
            
            clientStateBuffer.Add(rewindState, rewindState.tick);
            
            // Replay all inputs from the rewind state to the current state
            int tickToReplay = lastServerState.tick;

            while (tickToReplay < timer.CurrentTick) {
                int bufferIndex = tickToReplay % k_bufferSize;
                StatePayload statePayload = ProcessMovement(clientInputBuffer.Get(bufferIndex));
                clientStateBuffer.Add(statePayload, bufferIndex);
                tickToReplay++;
            }
        }
        
        [ServerRpc]
        void SendToServerRpc(InputPayload input) {
            serverInputQueue.Enqueue(input);
        }

        StatePayload ProcessMovement(InputPayload input) {
            Move(input.inputVector);
            
            return new StatePayload() {
                tick = input.tick,
                position = transform.position,
                rotation = transform.rotation,
                velocity = rb.velocity,
                angularVelocity = rb.angularVelocity
            };
        }

        void Move(Vector2 inputVector) {
            float verticalInput = AdjustInput(input.Move.y);
            float horizontalInput = AdjustInput(input.Move.x);
            
            float motor = maxMotorTorque * verticalInput;
            float steering = maxSteeringAngle * horizontalInput;

            UpdateAxles(motor, steering);
            UpdateBanking(horizontalInput);
            
            kartVelocity = transform.InverseTransformDirection(rb.velocity);

            if (IsGrounded) {
                HandleGroundedMovement(verticalInput, horizontalInput);
            } else {
                HandleAirborneMovement(verticalInput, horizontalInput);
            }
        }

        void HandleGroundedMovement(float verticalInput, float horizontalInput) {
            // Turn logic
            if (Mathf.Abs(verticalInput) > 0.1f || Mathf.Abs(kartVelocity.z) > 1) {
                float turnMultiplier = Mathf.Clamp01(turnCurve.Evaluate(kartVelocity.magnitude / maxSpeed));
                rb.AddTorque(Vector3.up * (horizontalInput * Mathf.Sign(kartVelocity.z) * turnStrength * 100f * turnMultiplier));
            }
            
            // Acceleration Logic
            if (!input.IsBraking) {
                float targetSpeed = verticalInput * maxSpeed;
                Vector3 forwardWithoutY = transform.forward.With(y: 0).normalized;
                float latencyFactor = IsHost ? 0.4f : 0.05f;
                float lerpFraction = timer.MinTimeBetweenTicks / (latencyFactor / Time.deltaTime);  
                rb.velocity = Vector3.Lerp(rb.velocity, forwardWithoutY * targetSpeed, lerpFraction);
            }
            
            // Downforce - always push the cart down, using lateral Gs to scale the force if the Kart is moving sideways fast
            float speedFactor = Mathf.Clamp01(rb.velocity.magnitude / maxSpeed);
            float lateralG = Mathf.Abs(Vector3.Dot(rb.velocity, transform.right));
            float downForceFactor = Mathf.Max(speedFactor, lateralG / lateralGScale);
            rb.AddForce(-transform.up * (downForce * rb.mass * downForceFactor));
            
            // Shift Center of Mass
            float speed = rb.velocity.magnitude;
            Vector3 centerOfMassAdjustment = (speed > thresholdSpeed) 
                ? new Vector3(0f, 0f, Mathf.Abs(verticalInput) > 0.1f ? Mathf.Sign(verticalInput) * centerOfMassOffset : 0f)
                : Vector3.zero;
            rb.centerOfMass = originalCenterOfMass + centerOfMassAdjustment;
        }

        void HandleAirborneMovement(float verticalInput, float horizontalInput) {
            // Apply gravity to the Kart while its airborne
            rb.velocity = Vector3.Lerp(rb.velocity, rb.velocity + Vector3.down * gravity, Time.deltaTime * gravity);
        }

        void UpdateBanking(float horizontalInput) {
            // Bank the Kart in the opposite direction of the turn
            float targetBankAngle = horizontalInput * -maxBankAngle;
            Vector3 currentEuler = transform.localEulerAngles;
            currentEuler.z = Mathf.LerpAngle(currentEuler.z, targetBankAngle, Time.deltaTime * bankSpeed);
            transform.localEulerAngles = currentEuler;
        }

        void UpdateAxles(float motor, float steering) {
            foreach (AxleInfo axleInfo in axleInfos) {
                HandleSteering(axleInfo, steering);
                HandleMotor(axleInfo, motor);
                HandleBrakesAndDrift(axleInfo);
                UpdateWheelVisuals(axleInfo.leftWheel);
                UpdateWheelVisuals(axleInfo.rightWheel);
            }
        }

        void UpdateWheelVisuals(WheelCollider collider) {
            if (collider.transform.childCount == 0) return;
            
            Transform visualWheel = collider.transform.GetChild(0);
            
            Vector3 position;
            Quaternion rotation;
            collider.GetWorldPose(out position, out rotation);
            
            visualWheel.transform.position = position;
            visualWheel.transform.rotation = rotation;
        }

        void HandleSteering(AxleInfo axleInfo, float steering) {
            if (axleInfo.steering) {
                float steeringMultiplier = input.IsBraking ? driftSteerMultiplier : 1f;
                axleInfo.leftWheel.steerAngle = steering * steeringMultiplier;
                axleInfo.rightWheel.steerAngle = steering * steeringMultiplier;
            }
        }
        
        void HandleMotor(AxleInfo axleInfo, float motor) {
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
        }

        void HandleBrakesAndDrift(AxleInfo axleInfo) {
            if (axleInfo.motor) {
                if (input.IsBraking) {
                    rb.constraints = RigidbodyConstraints.FreezeRotationX;

                    float newZ = Mathf.SmoothDamp(rb.velocity.z, 0, ref brakeVelocity, 1f);
                    rb.velocity = rb.velocity.With(z: newZ);
                    
                    axleInfo.leftWheel.brakeTorque = brakeTorque;
                    axleInfo.rightWheel.brakeTorque = brakeTorque;
                    ApplyDriftFriction(axleInfo.leftWheel);
                    ApplyDriftFriction(axleInfo.rightWheel);
                } else {
                    rb.constraints = RigidbodyConstraints.None;
                    
                    axleInfo.leftWheel.brakeTorque = 0;
                    axleInfo.rightWheel.brakeTorque = 0;
                    ResetDriftFriction(axleInfo.leftWheel);
                    ResetDriftFriction(axleInfo.rightWheel);
                }
            }
        }

        void ResetDriftFriction(WheelCollider wheel) {
            AxleInfo axleInfo = axleInfos.FirstOrDefault(axle => axle.leftWheel == wheel || axle.rightWheel == wheel);
            if (axleInfo == null) return;
            
            wheel.forwardFriction = axleInfo.originalForwardFriction;
            wheel.sidewaysFriction = axleInfo.originalSidewaysFriction;
        }

        void ApplyDriftFriction(WheelCollider wheel) {
            if (wheel.GetGroundHit(out var hit)) {
                wheel.forwardFriction = UpdateFriction(wheel.forwardFriction);
                wheel.sidewaysFriction = UpdateFriction(wheel.sidewaysFriction);
                IsGrounded = true;
            }
        }

        WheelFrictionCurve UpdateFriction(WheelFrictionCurve friction) {
            friction.stiffness = input.IsBraking ? Mathf.SmoothDamp(friction.stiffness, .5f, ref driftVelocity, Time.deltaTime * 2f) : 1f;
            return friction;
        }

        float AdjustInput(float input) {
            return input switch {
                >= .7f => 1f,
                <= -.7f => -1f,
                _ => input
            };
        }
    }
}