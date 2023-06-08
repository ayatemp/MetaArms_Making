using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


//3軸ロボットアームでの逆運動学の実装
namespace InverseKinematics
{
    public class JointController_X90r: MonoBehaviour
    {
        //robot
        private GameObject[] joint = new GameObject[3];
        private GameObject[] arm = new GameObject[3];
        private float[] armL = new float[3];
        private Vector3[] angle = new Vector3[3];

        //UI
        private GameObject[] slider = new GameObject[3];
        private float[] sliderVal = new float[3];
        private float[] prevSliderVal = new float[3];
        private GameObject[] angText = new GameObject[3];
        private GameObject[] posText = new GameObject[3];

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

            /*
            //ローカル座標系 
            //結局ばぐったのでワールド座標系でやることにした
            Vector3 localTargetPosition = joint[0].transform.InverseTransformPoint(new Vector3(sliderVal[0], sliderVal[1], sliderVal[2]));
            float x = localTargetPosition.x;
            float y = localTargetPosition.y;
            float z = localTargetPosition.z;
            */

            
            //ワールド座標系
            //sliderの値をワールド座標系に変換
            float x = sliderVal[0];
            float y = sliderVal[1];
            float z = sliderVal[2];
            
            //逆運動学の計算
            /*-----------------------------------------------------------------------------------------------------------------*/
            angle[0].y = -Mathf.Atan2(z,x);

            float a = x / Mathf.Cos(angle[0].y);
            float b = y - armL[0];

            if(Mathf.Pow(a*a +b*b,0.5f)>(armL[1]+armL[2]))
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
                float alfa = Mathf.Acos((armL[1]*armL[1]+armL[2]*armL[2]-a*a-b*b)/ (2f*armL[1]*armL[2]));
                angle[2].z = -Mathf.PI + alfa;

                float beta = Mathf.Acos((armL[1]*armL[1]+ a*a +b*b -armL[2]*armL[2])/(2f*armL[1]*Mathf.Sqrt(a*a+b*b)));
                angle[1].z = Mathf.Atan2(b,a) + beta;

                for(int i=0;i<joint.Length;i++)
                {
                    joint[i].transform.localEulerAngles = angle[i] * Mathf.Rad2Deg;
                    posText[i].GetComponent<Text>().text = sliderVal[i].ToString("f2");
                    prevSliderVal[i] = sliderVal[i];
                }
                //joint[0].transform.localEulerAngles = new Vector3(-90, joint[0].transform.localEulerAngles.y, joint[0].transform.localEulerAngles.z);

                angText[0].GetComponent<Text>().text = ( angle[0].z * Mathf.Rad2Deg).ToString("f2");
                angText[1].GetComponent<Text>().text = ( angle[1].z * Mathf.Rad2Deg).ToString("f2");
                angText[2].GetComponent<Text>().text = ( angle[1].z * Mathf.Rad2Deg).ToString("f2");
            }

            /*-----------------------------------------------------------------------------------------------------------------*/
        }
        
    }
}


