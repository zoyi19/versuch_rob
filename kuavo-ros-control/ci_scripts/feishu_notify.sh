#!/bin/bash

# Usage: bash feishu_notify.sh "message content" "webhook_url"

content="$1"
webhook_url="$2"

if [ -z "$webhook_url" ]; then
  echo "Feishu webhook not configured, skipping notification."
  exit 0
fi

curl -s "$webhook_url" -H 'Content-Type: application/json' -d "
   {
    \"msg_type\": \"text\",
    \"content\": {
        \"text\": \"通知 $content\"
    }
   }"
