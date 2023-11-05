using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class Game : MonoBehaviour
{
    public int port = 1234;  // 受信に使用するポート番号
    private UdpClient udpClient;
    private Vector3 lastPosition = Vector3.zero;  // 最後に受信した座標
    private Vector3 lastPosition2 = Vector3.zero;
    private Vector3 lastPosition3 = Vector3.zero;
    private Vector3 lastPosition4 = Vector3.zero;
    private Vector3 lastPosition5 = Vector3.zero;
    private Vector3 lastPosition6 = Vector3.zero;
    private Vector3 leftPosition = Vector3.zero;

    private Vector3 base_position = Vector3.zero;
    //public Transform ball;
    //public Transform real;
    //public Transform goal;

    public GameObject ball;
    public GameObject real;
    public GameObject goal;
    public GameObject test;
    //private GameObject target;
    //public Transform leftHandModel;
    //public Transform rightHandModel;

    public GameObject realb;
    public GameObject goalb;

    public Transform leftHandAnchor;
    public Transform rightHandAnchor;
    private void Start()
    {
        ball = GameObject.Find("Ball");
        real = GameObject.Find("Real");
        goal = GameObject.Find("Goal");
        test = GameObject.Find("System/Offset/Test");
        realb = GameObject.Find("Realb");
        goalb = GameObject.Find("Goalb");


        // UdpClientの初期
        udpClient = new UdpClient(port);

        //transform.position = new Vector3(0, 0, 0);
        real.transform.position = new Vector3(5, 2, 0);
        goal.transform.position = new Vector3(4, 2, 0);
        realb.transform.position = new Vector3(5, 2, 0);
        goalb.transform.position = new Vector3(4, 2, 0);
        test.transform.position = new Vector3(3, 0, -3);
        lastPosition = ball.transform.position;  // 最後に受信した座標を更新
        lastPosition2 = goal.transform.position;
        lastPosition3 = real.transform.position;
        lastPosition4 = test.transform.position; // 最後に受信した座標を更新
        lastPosition5 = goalb.transform.position;
        lastPosition6 = realb.transform.position;


        //leftPosition = target.transform.position;
        //Debug.Log(leftPosition);




    }

    private void Update()
    {

        //Vector3 leftControllerPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);

        // 右のコントローラのボタンが押されたかを確認
        //bool isRightControllerButtonPressed = OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch);

        //if (isRightControllerButtonPressed)
        //{
           // base_position = leftControllerPosition;
        //}


        // UDPデータの受信
        if (udpClient.Available > 0)
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
            byte[] data = udpClient.Receive(ref remoteEndPoint);
            string message = Encoding.UTF8.GetString(data);

            // 受信したデータから座標位置を解析
            string[] coordinates = message.Split(',');
            if (coordinates.Length == 11 && float.TryParse(coordinates[0], out float x) &&
                float.TryParse(coordinates[1], out float y) && float.TryParse(coordinates[2], out float z) && float.TryParse(coordinates[3], out float x2) &&
                float.TryParse(coordinates[4], out float y2) && float.TryParse(coordinates[5], out float z2) && float.TryParse(coordinates[6], out float x3) &&
                float.TryParse(coordinates[7], out float y3) && float.TryParse(coordinates[8], out float z3) && float.TryParse(coordinates[9], out float disp) && float.TryParse(coordinates[10], out float dx))
            {
                // ボールの座標を更新
                ball.transform.position = new Vector3(x, y, z);
                goal.transform.position = new Vector3(x2, y2, z2);
                real.transform.position = new Vector3(x3, y3, z3);
                goalb.transform.position = new Vector3(x2, 0, 3);
                realb.transform.position = new Vector3(x3, 0, 3);
                //Vector3 leftPosition = leftHandAnchor.position;
                //leftPosition = target.transform.position;
                //Quaternion leftRotation = leftHandAnchor.rotation;

                // 右手のコントローラの位置と回転を取得
                //Vector3 rightPosition = rightHandAnchor.position;
                //Quaternion rightRotation = rightHandAnchor.rotation;
                if (disp == 1)
                {
                    Vector3 upPosition = base_position;
                    upPosition.x += dx;

                    test.transform.position = upPosition;
                }
                lastPosition = ball.transform.position;  // 最後に受信した座標を更新
                lastPosition2 = goal.transform.position;
                lastPosition3 = real.transform.position;
                lastPosition5 = goalb.transform.position;
                lastPosition6 = realb.transform.position;
                lastPosition4 = test.transform.position;
            }
            else
            {
                Debug.Log("error");
            }
        }
        else
        {
            // 受信していない場合は最後に受信した座標でボールの位置を更新
            Debug.Log("no");
            ball.transform.position = lastPosition;
            goal.transform.position = lastPosition2;
            real.transform.position = lastPosition3;
            test.transform.position = lastPosition4;
            goalb.transform.position = lastPosition5;
            realb.transform.position = lastPosition6;

        }
    }

    private void OnDestroy()
    {
        // UdpClientのクローズ
        udpClient.Close();
    }
}