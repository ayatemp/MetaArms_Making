using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using MathNet.Numerics.LinearAlgebra.Single;


//3軸ロボットアームでの逆運動学の実装
namespace InverseKinematics
{
    public class JointController_6_axis: MonoBehaviour
    {
        //robot
        private GameObject[] joint = new GameObject[6];
        private GameObject[] arm = new GameObject[6];
        private float[] armL = new float[6];
        private Vector3[] angle = new Vector3[6];
        private Vector3 pos;
        private Vector3 rot;

        //UI
        private GameObject[] slider = new GameObject[6];
        private float[] sliderVal = new float[6];
        private float[] prevSliderVal = new float[6];
        private GameObject[] angText = new GameObject[6];
        private GameObject[] posText = new GameObject[6];

        void Start()
        {
            //check();
            //robot
            for(int i = 0; i<joint.Length; i++)
            {
                joint[i] = GameObject.Find("Joint_"+i.ToString());
                arm[i] = GameObject.Find("Arm_"+i.ToString());
                if(i==0) armL[i] = arm[i].transform.localScale.y;
                else armL[i] = arm[i].transform.localScale.x;
            }
            
            //ベースの回転の修正
            //結局ダメだった
            //joint[0].transform.Rotate(-90, 0, 0);

            //UIsetting
            for(int i =0; i<joint.Length;i++)
            {
                slider[i] = GameObject.Find("Slider_"+i.ToString());
                sliderVal[i]= slider[i].GetComponent<Slider>().value;

                posText[i]=GameObject.Find("Ref_"+i.ToString());
                angText[i]=GameObject.Find("Ang_"+i.ToString());
                
            }
        }

        void Update()
        {
            for(int i = 0; i<joint.Length; i++)
            {
                sliderVal[i]= slider[i].GetComponent<Slider>().value;
            }
            pos.x = sliderVal[0];
            pos.y = sliderVal[1];
            pos.z = sliderVal[2];
            rot.x = sliderVal[3];
            rot.y = sliderVal[4];
            rot.z = sliderVal[5];
            
            float endL = armL[4] + armL[5];
            Quaternion q = Quaternion.Euler(rot.x,rot.y,rot.z);
            Vector3 v = new Vector3(endL,0f,0f);
            float x = pos.x -(q*v).x;
            float y = pos.y -(q*v).y;
            float z = pos.z -(q*v).z;
            
            //逆運動学の計算
            /*-----------------------------------------------------------------------------------------------------------------*/
            angle[0].y = -Mathf.Atan2(z,x);

            float a = x / Mathf.Cos(angle[0].y);
            float b = y - armL[0];

            if(Mathf.Pow(a*a +b*b,0.5f)> (armL[1] + armL[2] + armL[3]))
            {
                for(int i = 0; i<joint.Length; i++)
                {
                    sliderVal[i] = prevSliderVal[i];
                    slider[i].GetComponent<Slider>().value = sliderVal[i];
                }
                return;
            }
            else
            {
                float alfa = Mathf.Acos((armL[1]*armL[1]+(armL[2]+armL[3])*(armL[2]+armL[3])-a*a-b*b)/ (2f*armL[1]*(armL[2]+armL[3])));
                angle[2].z = -Mathf.PI + alfa;

                float beta = Mathf.Acos((armL[1]*armL[1]+ a*a +b*b -(armL[2]+armL[3])*(armL[2]+armL[3]))/(2f*armL[1]*Mathf.Sqrt(a*a+b*b)));
                angle[1].z = Mathf.Atan2(b,a) + beta;

                // cale angle[3],[4],[5]
                var R0_3 = CalcR0_3();
                var R0_6 = CalcR0_6();
                var R3_6 = R0_3.Inverse() * R0_6;
                angle[4].z = Mathf.Acos(R3_6[2,2]);
                angle[5].x = Mathf.Acos(-R3_6[2,0] / (Mathf.Sin(angle[4].z)+ 1.0e-6f));
                angle[3].x = Mathf.Asin(R3_6[1,2] / (Mathf.Sin(angle[4].z)+ 1.0e-6f));

                //error check
                if((R3_6[2,1] * (Mathf.Sin(angle[4].z)* Mathf.Sin(angle[5].x))) < 0)
                {
                    angle[5].x = -Mathf.Acos(-R3_6[2,0] / (Mathf.Sin(angle[4].z)+ 1.0e-6f));
                }
                if((R3_6[0,2] * (Mathf.Cos(angle[3].x)* Mathf.Sin(angle[4].z))) < 0)
                {
                    angle[3].x = -angle[3].x + Mathf.PI;
                }

                //correct angles
                angle[3].x = -angle[3].x;
                //angle[4].z = -angle[4].z;
                angle[5].x = -angle[5].x;

                for(int i=0;i<joint.Length;i++)
                {
                    joint[i].transform.localEulerAngles = angle[i] * Mathf.Rad2Deg;
                    posText[i].GetComponent<Text>().text = sliderVal[i].ToString("f2");
                    prevSliderVal[i] = sliderVal[i];
                }
                //joint[0].transform.localEulerAngles = new Vector3(-90, joint[0].transform.localEulerAngles.y, joint[0].transform.localEulerAngles.z);

                angText[0].GetComponent<Text>().text = ( angle[0].y * Mathf.Rad2Deg).ToString("f2");
                angText[1].GetComponent<Text>().text = ( angle[1].z * Mathf.Rad2Deg).ToString("f2");
                angText[2].GetComponent<Text>().text = ( angle[1].z * Mathf.Rad2Deg).ToString("f2");
                angText[3].GetComponent<Text>().text = ( angle[3].x * Mathf.Rad2Deg).ToString("f2");    
                angText[4].GetComponent<Text>().text = ( angle[4].z * Mathf.Rad2Deg).ToString("f2");
                angText[5].GetComponent<Text>().text = ( angle[5].x * Mathf.Rad2Deg).ToString("f2");
            }

            /*-----------------------------------------------------------------------------------------------------------------*/
        }
        
        DenseMatrix CalcR0_3()
        {
            float[] C = new float[6];
            float[] S = new float[6];
            C[0] = Mathf.Cos(-angle[0].y);
            C[1] = Mathf.Cos(angle[1].z);
            C[2] = Mathf.Cos(angle[2].z);
            S[0] = Mathf.Sin(-angle[0].y);
            S[1] = Mathf.Sin(angle[1].z);
            S[2] = Mathf.Sin(angle[2].z);
            var R0_1 = DenseMatrix.OfArray(new float[,]
            {
                { C[0], 0f, S[0]},
                { S[0], 0f,-C[0]},
                { 0f, 1f, 0f}
            });
            var R1_2 = DenseMatrix.OfArray(new float[,]
            {
                { C[1], -S[1],0f},
                { S[1], C[1],0f},
                { 0f, 0f, 1f}
            });
            var R2_3 = DenseMatrix.OfArray(new float[,]
            {
                { -S[2],0f,C[2]},
                { C[2],0f,S[2]},
                { 0f, 1f, 0f}
            });

            return R0_1 * R1_2 * R2_3;
        }

        DenseMatrix CalcR0_6()
        {
            float rx = rot.x * Mathf.Deg2Rad;
            float ry = rot.y * Mathf.Deg2Rad;
            float rz = rot.z * Mathf.Deg2Rad;

            var R0_6z = DenseMatrix.OfArray(new float[,]
            {
                { Mathf.Cos(-ry), -Mathf.Sin(-ry),0f},
                { Mathf.Sin(-ry), Mathf.Cos(-ry),0f},
                { 0f, 0f, 1f}
            });
            var R0_6y = DenseMatrix.OfArray(new float[,]
            {
                { Mathf.Cos(-rz), 0f, Mathf.Sin(-rz)},
                { 0f, 1f, 0f},
                { -Mathf.Sin(-rz), 0f, Mathf.Cos(-rz)}
            });
            var R0_6x = DenseMatrix.OfArray(new float[,]
            {
                { 1f, 0f, 0f},
                { 0f, Mathf.Cos(-rx), -Mathf.Sin(-rx)},
                { 0f, Mathf.Sin(-rx), Mathf.Cos(-rx)}
            });
            var baseRot = DenseMatrix.OfArray(new float[,]
            {
                { 0f, 0f, 1f},
                { 0f, -1f, 0f},
                { 1f, 0f, 0f}
            });

            return R0_6z * R0_6x * R0_6y * baseRot;
        }
    }
}


