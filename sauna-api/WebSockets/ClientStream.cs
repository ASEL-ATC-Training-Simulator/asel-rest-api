using Microsoft.AspNetCore.SignalR;
using SaunaSim.Api.ApiObjects.Aircraft;
using SaunaSim.Api.ApiObjects.Server;
using SaunaSim.Api.WebSockets.ResponseData;
using SaunaSim.Core.Data;
using SaunaSim.Core.Simulator.Aircraft;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net.WebSockets;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;
using System.Threading.Tasks;

namespace SaunaSim.Api.WebSockets
{
    public class ClientStream
    {
        private WebSocket _ws;
        private bool _cancellationRequested;
        private Queue<ISocketResponseData> _responseQueue;
        private SemaphoreSlim _responseQueueLock;
        private Task _sendWorker;

        public bool ShouldClose { get => _cancellationRequested; set => _cancellationRequested = value; }

        public ClientStream(WebSocket ws)
        {
            _ws = ws;
            _cancellationRequested = false;
            _responseQueue = new Queue<ISocketResponseData>();
            _responseQueueLock = new SemaphoreSlim(1);
        }

        private async Task ResponseWorker()
        {
            while (!_cancellationRequested && _ws.State == WebSocketState.Open)
            {
                await _responseQueueLock.WaitAsync();
                bool found = _responseQueue.TryDequeue(out var msg);
                _responseQueueLock.Release();
                if (found)
                {
                    await SendObject(msg);
                } else
                {
                    await Task.Delay(AppSettingsManager.PosCalcRate);
                }
            }
        }

        private async Task SendString(string msg)
        {
            if (!_cancellationRequested && _ws.State == WebSocketState.Open)
            {
                var bytes = Encoding.UTF8.GetBytes(msg);
                var arraySegment = new ArraySegment<byte>(bytes, 0, bytes.Length);
                await _ws.SendAsync(arraySegment, WebSocketMessageType.Text, true, CancellationToken.None);
            }
        }

        public async Task QueueMessage(ISocketResponseData data)
        {
            await _responseQueueLock.WaitAsync();
            _responseQueue.Enqueue(data);
            _responseQueueLock.Release();
        }

        private async Task SendObject(ISocketResponseData data)
        {
            // Convert to JSON String
            string jsonString = string.Empty;

            using (var stream = new MemoryStream())
            {
                var options = new JsonSerializerOptions();
                options.Converters.Add(new JsonStringEnumConverter());
                options.PropertyNamingPolicy = JsonNamingPolicy.CamelCase;
                options.NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals;
                await JsonSerializer.SerializeAsync(stream, data, options);
                stream.Position = 0;
                using var reader = new StreamReader(stream);
                jsonString = await reader.ReadToEndAsync();
            }

            // Convert to byte array
            await SendString(jsonString);
        }

        public void StartSend()
        {
            _cancellationRequested = false;
            _sendWorker = Task.Run(ResponseWorker);
        }

        public void StopSend()
        {
            _cancellationRequested = true;
            _sendWorker.Wait();
        }
    }
}