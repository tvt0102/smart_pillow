const mqtt = require('mqtt');
const axios = require('axios');
const fs = require("fs");
var ip = '192.168.1.31';
const host = '192.168.1.18';
const port = 1883;
const clientId = `mqtt_${Math.random().toString(16).slice(3)}`
const connectUrl = `mqtt://${host}:${port}`
const client = mqtt.connect(connectUrl, {
  clientId,
  clean: true,
  connectTimeout: 4000,
  reconnectPeriod: 1000,
})
const topicPCG = 'message/nameFilePCG';
const topicPPG = 'message/nameFilePPG';
var nameFilePCG = '';
var nameFilePPG = '';
function callAPIPCG() {
    return Promise.all([
        axios.get(`http://${ip}/${nameFilePCG}.bin`,{ responseType: 'arraybuffer' })
        .then(response => {
            fs.writeFile("./uploads/dataINMP.bin",Buffer.from(response.data), (err)=>{
                if (err) {
                    console.error('Lỗi khi ghi file:', err);
                } else {  
                    console.log('Dữ liệu đã được lưu vào file dataINMP.bin');
                }
            })
            fs.appendFileFile("./uploads/data_all_INMP.bin", Buffer.from(response.data), (err) => {
                if (err) {
                console.error('Lỗi khi ghi file:', err);
                } else {
                console.log('Dữ liệu đã được thêm vào file dataINMP.txt');
                }
            });
        })
        .catch(error => {
            console.error(error);
        }),
        axios.post(`http://${ip}/delete/${nameFilePCG}.bin`, "")
        .then(response => {
            console.log('Đã xóa dữ liệu INMP thành công:');
        })
        .catch(error => {
            console.error('Lỗi khi gửi dữ liệu:', error);
        })
        
    ]);
  }   
//   function callAPIPPG() {
//     return Promise.all([
//         axios.get(`http://${ip}/${nameFilePPG}.txt`)
//         .then(response => {
//             fs.writeFile("./uploads/testMAX.txt",response.data, (err)=>{
//                 if (err) {
//                     console.error('Lỗi khi ghi file:', err);
//                 } else {  
//                     console.log('Dữ liệu đã được lưu vào file testMAX.txt');
//                 }
//             })
//             fs.appendFile("./uploads/dataMAX.txt", response.data, (err) => {
//                 if (err) {
//                 console.error('Lỗi khi ghi file:', err);
//                 } else {
//                 console.log('Dữ liệu đã được thêm vào file dataMAX.txt');
//                 }
//             });
//         })
//         .catch(error => {
//             console.error(error);
//         }),
//         axios.post(`http://${ip}/delete/${nameFilePPG}.txt`, "")
//         .then(response => {
//             console.log('Đã xóa dữ liệu MAX thành công:');
//         })
//         .catch(error => {
//             console.error('Lỗi khi gửi dữ liệu:');
//         })
        
//     ]);
//   }   


async function measureTimePCG(){
    let startTime = Date.now();
    startTime = startTime / 1000;    //time s
    await callAPIPCG();
    let endTime = Date.now();
    endTime = endTime / 1000;   

    console.log("Time run API of INMP: ",endTime - startTime);
    // Gửi lại thông báo xác nhận
    client.publish("message/fileAck", JSON.stringify({ file_ack: nameFilePCG }));
}

// async function measureTimePPG(){
//     let startTime = Date.now();
//     startTime = startTime / 1000;    //time s
//     await callAPIPPG();
//     let endTime = Date.now();
//     endTime = endTime / 1000;   

//     console.log("Time run API of MAX: ",endTime - startTime);
//     client.publish("message/fileAck", JSON.stringify({ file_ack: nameFilePPG }));
// }


client.on('connect', () => {
  console.log('Connected')
})

client.on('message',async function (topic, message) {
    // called each time a message is received
    
    message = JSON.parse(message.toString('utf-8'));
    console.log('Received message:', message);
    if(message.namefile.substring(0,3) == "PCG"){
        nameFilePCG = message.namefile;
        measureTimePCG();
    }
    // else{
    //     nameFilePPG = message.namefile;
    //     measureTimePPG();
    // }
    // nameFilePCG  = message.namefile;
    // console.log(nameFilePCG);
    //measureTimePCG(); 
});

client.subscribe(topicPCG);
//client.subscribe(topicPPG);